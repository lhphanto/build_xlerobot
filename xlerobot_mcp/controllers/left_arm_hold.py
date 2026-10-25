#!/usr/bin/env python3
"""Supervised preload-and-hold or supported release of left arm IDs1-6 only."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc
IDS=tuple(range(1,7))

def snapshot(sdk,ph,pk):
    return {str(m):{key:jc.read(sdk,ph,pk,m,key) for key in jc.REGS} for m in IDS}

def validate(group,states,expected):
    if group.get('port_verified') is not True or set(expected)!=set(map(str,IDS)):
        raise ValueError('Verified left bus and all6 expected positions required')
    for mid,s in states.items():
        joint=group['joints'][mid];limits=joint['safe_raw_limits']
        if s['model']!=777 or s['mode']!=0 or s['moving'] or s['torque']!=0:
            raise ValueError(f'ID{mid} must be stationary, position-mode, torqueOFF STS3215')
        if [s['min'],s['max']]!=joint['observed_raw_limits'] or not s['min']<=limits[0]<limits[1]<=s['max']:
            raise ValueError(f'ID{mid} calibration changed')
        if not limits[0]<=s['position']<=limits[1] or abs(s['position']-expected[mid])>8:
            raise ValueError(f'ID{mid} outside saved limits or pose changed')
        if not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:
            raise ValueError(f'ID{mid} voltage/temperature unsuitable')

def control(sdk,ph,pk,group,action,expected):
    if action=='recover-hold':
        from right_arm_hold import control as shared_hold
        return shared_hold(sdk,ph,pk,group,action,expected,left_startup=True)
    states=snapshot(sdk,ph,pk);enabled=[]
    def torque(motor,value):
        comm,error=pk.write1ByteTxRx(ph,motor,40,value)
        if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} torque acknowledgment failed')
        if jc.read(sdk,ph,pk,motor,'torque')!=value:raise RuntimeError(f'ID{motor} torque readback mismatch')
    if action=='release':
        if not group.get('port_verified') or any(s['model']!=777 or s['mode']!=0 or s['moving'] for s in states.values()):
            raise ValueError('Requires verified stationary left arm; user must support all links')
        failures=[]
        for motor in IDS:
            try:torque(motor,0)
            except Exception as exc:failures.append(str(exc))
        if failures:raise RuntimeError('Partial release: '+'; '.join(failures))
        return {'completed':True,'action':'release','states':snapshot(sdk,ph,pk)}
    if action!='hold':raise ValueError('Invalid action')
    validate(group,states,expected)
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
            if not lo<=start<=hi:raise RuntimeError('Current activation position outside limits')
            # Bytes42–47: position, zero time/PWM field, conservative speed.
            # No separate speed write while an old/stale position goal exists.
            packet=[start & 255,(start >> 8) & 255,0,0,100,0]
            enabled.append(motor)  # write acknowledgment can be uncertain
            comm,error=pk.writeTxRx(ph,motor,42,6,packet)
            if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} atomic hold acknowledgment failed')
            if jc.read(sdk,ph,pk,motor,'goal')!=start:
                raise RuntimeError('Current-position target verification failed')
            starts[str(motor)]=start
            current_torque=jc.read(sdk,ph,pk,motor,'torque')
            activation.append({'id':motor,'torque_after_target_write':current_torque})
            if current_torque==0:torque(motor,1)
            elif current_torque!=1:raise RuntimeError('Unexpected torque state after target write')
            time.sleep(.15)
            for other in IDS:
                s={k:jc.read(sdk,ph,pk,other,k) for k in ['position','voltage_present_raw','temperature_raw','moving']}
                if abs(s['position']-starts[str(other)])>8 or not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:
                    raise RuntimeError('Pose/power changed while enabling holding torque')
        final=snapshot(sdk,ph,pk)
        if any(s['torque']!=1 or s['moving'] or abs(s['position']-starts[m])>8 for m,s in final.items()):
            raise RuntimeError('Final stable holding state not confirmed')
        return {'completed':True,'action':'hold','preloaded_positions':starts,'activation':activation,'states':final,
                'note':'Holding test only; supported by operator. Supply current capacity remains unverified.'}
    except BaseException:
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
    config=json.loads(jc.CONFIG.read_text());group=config['groups']['left-arm'];port=group['port']
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
