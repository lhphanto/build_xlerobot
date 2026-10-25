#!/usr/bin/env python3
"""Supervised preload-and-hold or supported release of right arm IDs1-6 only."""
import copy
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc
IDS=tuple(range(1,7))

def snapshot(sdk,ph,pk):
    return {str(m):{key:jc.read(sdk,ph,pk,m,key) for key in jc.REGS} for m in IDS}

def validate(group,states,expected,recover=False,recovery_bound=16):
    if group.get('port_verified') is not True or set(expected)!=set(map(str,IDS)):
        raise ValueError('Verified right bus and all6 expected positions required')
    for mid,s in states.items():
        joint=group['joints'][mid];limits=joint['safe_raw_limits']
        if s['model']!=777 or s['mode']!=0 or s['moving'] or s['torque']!=0:
            raise ValueError(f'ID{mid} must be stationary, position-mode, torqueOFF STS3215')
        if [s['min'],s['max']]!=joint['observed_raw_limits'] or not s['min']<=limits[0]<limits[1]<=s['max']:
            raise ValueError(f'ID{mid} calibration changed')
        allowed=(recover and mid=='3' and limits[0]-recovery_bound<=s['position']<=limits[1]+recovery_bound) or limits[0]<=s['position']<=limits[1]
        if not allowed or abs(s['position']-expected[mid])>8:
            raise ValueError(f'ID{mid} outside saved limits or pose changed')
        if not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:
            raise ValueError(f'ID{mid} voltage/temperature unsuitable')

def control(sdk,ph,pk,group,action,expected,*,left_startup=False):
    if type(left_startup) is not bool or (left_startup and action!='recover-hold'):raise ValueError('Fixed left startup recovery profile only')
    outside,inset,max_travel,settle=(32,32,64,24) if left_startup else (16,16,32,4)
    # Hold-only local window for unqualified ID5. Never persist/approve travel limits.
    group=copy.deepcopy(group)
    if action=='hold' and group['joints']['5']['safe_raw_limits'] is None:
        if set(expected)!=set(map(str,IDS)) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):
            raise ValueError('All6 integer expected positions required')
        if group['joints']['5']['observed_raw_limits'] != [0,4095]:
            raise ValueError('ID5 unqualified limits changed')
        center=expected['5']
        if not 8<=center<=4087:raise ValueError('ID5 hold pose too close to endpoint')
        group['joints']['5']['safe_raw_limits']=[center-8,center+8]
    states=snapshot(sdk,ph,pk);enabled=[]
    def torque(motor,value):
        comm,error=pk.write1ByteTxRx(ph,motor,40,value)
        if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} torque acknowledgment failed')
        if jc.read(sdk,ph,pk,motor,'torque')!=value:raise RuntimeError(f'ID{motor} torque readback mismatch')
    if action=='release':
        if not group.get('port_verified') or any(s['model']!=777 or s['mode']!=0 or s['moving'] for s in states.values()):
            raise ValueError('Requires verified stationary right arm; user must support all links')
        failures=[]
        for motor in IDS:
            try:torque(motor,0)
            except Exception as exc:failures.append(str(exc))
        if failures:raise RuntimeError('Partial release: '+'; '.join(failures))
        return {'completed':True,'action':'release','states':snapshot(sdk,ph,pk)}
    if action not in ('hold','recover-hold'):raise ValueError('Invalid action')
    recover=action=='recover-hold'
    validate(group,states,expected,recover,outside)
    if left_startup:
        lo,hi=group['joints']['3']['safe_raw_limits'];position=states['3']['position']
        if not lo<=position<=hi:
            target=lo+inset if position<lo else hi-inset
            if not lo+8<=target<=hi-8 or abs(target-position)>max_travel:raise ValueError('No bounded interior recovery target; no activation')
    starts={m:s['position'] for m,s in states.items()}
    try:
        activation=[]
        # A target write may enable torque on this hardware. Treat this atomic
        # current-target/speed packet as the activation boundary for ONE joint.
        for motor in IDS:
            for other in IDS:
                allowed=8 if other in enabled else 4
                if abs(jc.read(sdk,ph,pk,other,'position')-starts[str(other)])>allowed:
                    raise RuntimeError('Supported pose changed; stop enabling joints')
            start=jc.read(sdk,ph,pk,motor,'position')
            lo,hi=group['joints'][str(motor)]['safe_raw_limits']
            recovering=recover and motor==3 and not lo<=start<=hi
            if not lo<=start<=hi and not recovering:raise RuntimeError('Current activation position outside limits')
            if recovering and not lo-outside<=start<=hi+outside:raise RuntimeError('Elbow drift exceeds recovery bound')
            target=lo+inset if recovering and start<lo else hi-inset if recovering else start
            if not lo<=target<=hi or abs(target-start)>max_travel:raise RuntimeError('Inward recovery target invalid')
            # Bytes42–47: position, zero time/PWM field, conservative speed.
            # No separate speed write while an old/stale position goal exists.
            packet=[target & 255,(target >> 8) & 255,0,0,30 if recovering else 100,0]
            enabled.append(motor)  # write acknowledgment can be uncertain
            comm,error=pk.writeTxRx(ph,motor,42,6,packet)
            if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} atomic hold acknowledgment failed')
            if jc.read(sdk,ph,pk,motor,'goal')!=target:
                raise RuntimeError('Current-position target verification failed')
            starts[str(motor)]=target
            current_torque=jc.read(sdk,ph,pk,motor,'torque')
            activation.append({'id':motor,'torque_after_target_write':current_torque})
            if current_torque==0:torque(motor,1)
            elif current_torque!=1:raise RuntimeError('Unexpected torque state after target write')
            if recovering:
                deadline=time.monotonic()+(3.5 if left_startup else 2.5);stable=0
                while time.monotonic()<deadline:
                    current=snapshot(sdk,ph,pk)
                    for mid,c in current.items():
                        j=group['joints'][mid]
                        if c['model']!=777 or c['mode']!=0 or [c['min'],c['max']]!=j['observed_raw_limits'] or c['torque']!=(1 if int(mid) in enabled else 0) or not 110<=c['voltage_present_raw']<=130 or c['temperature_raw']>=60:raise RuntimeError('State/power changed during inward recovery')
                        if mid=='3':
                            corridor=(target-4,start+2) if start>hi else (start-2,target+4)
                            if c['goal']!=target or not corridor[0]<=c['position']<=corridor[1]:raise RuntimeError('Elbow escaped inward recovery corridor')
                        elif not j['safe_raw_limits'][0]<=c['position']<=j['safe_raw_limits'][1] or c['moving'] or abs(c['position']-starts[mid])>8:raise RuntimeError('Another joint moved during inward recovery')
                    c=current['3'];margin=8 if left_startup else 0
                    stable=stable+1 if lo+margin<=c['position']<=hi-margin and abs(c['position']-target)<=settle and not c['moving'] else 0
                    if stable>=3:break
                    time.sleep(.05)
                else:raise RuntimeError('Elbow did not enter and settle inside saved range; no next activation')
                activation[-1].update({'inward_recovery':True,'start_raw':start,'target_raw':target,'speed_raw':30,'final_position':c['position']})
                if left_startup and abs(c['position']-target)>8:
                    # Explicit startup handoff after verified IN-RANGE settling,
                    # never a timeout retry or an out-of-range hold target.
                    position=jc.read(sdk,ph,pk,motor,'position')
                    if not lo+8<=position<=hi-8 or abs(position-c['position'])>4:raise RuntimeError('Recovered pose changed before holding handoff')
                    packet=[position & 255,(position >> 8) & 255,0,0,30,0]
                    comm,error=pk.writeTxRx(ph,motor,42,6,packet)
                    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Recovery holding handoff acknowledgment failed; no retry')
                    if jc.read(sdk,ph,pk,motor,'goal')!=position or jc.read(sdk,ph,pk,motor,'torque')!=1:raise RuntimeError('Recovery holding handoff readback failed')
                    starts[str(motor)]=position
                    activation[-1]['holding_handoff_target']=position

            time.sleep(.15)
            for other in IDS:
                s={k:jc.read(sdk,ph,pk,other,k) for k in ['position','voltage_present_raw','temperature_raw','moving']}
                if abs(s['position']-starts[str(other)])>8 or not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:
                    raise RuntimeError('Pose/power changed while enabling holding torque')
        final=snapshot(sdk,ph,pk)
        if any(s['torque']!=1 or s['moving'] or s['goal']!=starts[m] or abs(s['position']-starts[m])>8 or not group['joints'][m]['safe_raw_limits'][0]<=s['position']<=group['joints'][m]['safe_raw_limits'][1] for m,s in final.items()):
            raise RuntimeError('Final stable holding state not confirmed')
        if left_startup and any(item.get('inward_recovery') for item in activation):
            lo,hi=group['joints']['3']['safe_raw_limits']
            if not lo+8<=final['3']['position']<=hi-8:raise RuntimeError('Recovered elbow lost inside-range margin; hold remains partial')
        return {'completed':True,'action':action,'preloaded_positions':starts,'activation':activation,'states':final,
                'note':'Holding test only; supported by operator. Supply current capacity remains unverified.'}
    except BaseException:
        if recover:
            # Do not issue an out-of-range current-position goal or automatically
            # release gravity support. Existing recovery target stays inward.
            print('Recovery/hold failed; activation may be partial. Existing inward/hold targets retained. Keep full support and use cutoff if unexpected. No automatic retry.',file=sys.stderr)
            raise
        old={s:signal.signal(s,signal.SIG_IGN) for s in (signal.SIGINT,signal.SIGTERM)}
        try:
            for motor in enabled:
                try:
                    if jc.read(sdk,ph,pk,motor,'torque')!=1:continue
                    pos=jc.read(sdk,ph,pk,motor,'position');lo,hi=group['joints'][str(motor)]['safe_raw_limits']
                    if not lo<=pos<=hi:raise RuntimeError('Current hold feedback outside limits')
                    jc.write(sdk,ph,pk,motor,42,pos)
                except Exception as exc:print(f'HOLD FAILED ID{motor}: {exc}',file=sys.stderr)
            print('Holding state may be partial. Keep arm supported; no automatic torque release.',file=sys.stderr)
        finally:
            for sig,handler in old.items():signal.signal(sig,handler)
        raise

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['hold','release','recover-hold']);p.add_argument('--expected-positions',default='{}');a=p.parse_args()
    config=json.loads(jc.CONFIG.read_text());group=config['groups']['right-arm'];port=group['port']
    import scservo_sdk as sdk
    name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            print(json.dumps(control(sdk,ph,pk,group,a.action,json.loads(a.expected_positions))),flush=True)
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
