#!/usr/bin/env python3
"""Enable holding on neck IDs7-8 at their current pose: preload each goal, then torque; no commanded travel."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc

IDS=('8','7')  # Tilt first: it carries the camera against gravity.


def control(sdk,ph,pk,group,expected):
    if group.get('port_verified') is not True or set(expected)!=set(IDS):raise ValueError('Verified neck bus and both expected positions required')
    states={mid:{k:jc.read(sdk,ph,pk,int(mid),k) for k in jc.REGS} for mid in IDS}
    # Check everything before the first write, so a bad joint leaves both untouched.
    for mid,s in states.items():
        joint=group['joints'][mid];lo,hi=joint['safe_raw_limits']
        if joint.get('commissioned') is not True or s['model']!=777 or s['mode']!=0 or s['moving']:raise ValueError(f'ID{mid} must be a commissioned, stationary position-mode STS3215')
        if [s['min'],s['max']]!=joint['observed_raw_limits'] or not lo<=s['position']<=hi:raise ValueError(f'ID{mid} calibration changed or outside operating limits')
        if abs(s['position']-expected[mid])>8 or not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:raise ValueError(f'ID{mid} pose changed or power/temperature unsuitable')
        if s['torque']==1 and abs(s['goal']-s['position'])>8:raise ValueError(f'ID{mid} has an outstanding target; refusing takeover')
    enabled=[]
    for mid in IDS:
        if states[mid]['torque']==1:continue
        motor,start=int(mid),states[mid]['position']
        jc.write(sdk,ph,pk,motor,44,0);jc.write(sdk,ph,pk,motor,46,100);jc.write(sdk,ph,pk,motor,42,start)
        if jc.read(sdk,ph,pk,motor,'goal')!=start or abs(jc.read(sdk,ph,pk,motor,'position')-start)>4:raise RuntimeError(f'ID{mid} preload failed or neck still moving; no torque enabled on it')
        comm,error=pk.write1ByteTxRx(ph,motor,40,1)
        if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{mid} torque write acknowledgment failed; no retry')
        if jc.read(sdk,ph,pk,motor,'torque')!=1:raise RuntimeError(f'ID{mid} torque readback differs')
        time.sleep(.3)
        if abs(jc.read(sdk,ph,pk,motor,'position')-start)>8:raise RuntimeError(f'ID{mid} moved after enabling hold; inspect before continuing')
        enabled.append(motor)
    return {'completed':True,'enabled':enabled,'torque':'neck on',
            'positions':{mid:jc.read(sdk,ph,pk,int(mid),'position') for mid in IDS},'torques':{mid:jc.read(sdk,ph,pk,int(mid),'torque') for mid in IDS}}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--expected-positions',required=True);a=p.parse_args()
    expected=json.loads(a.expected_positions)
    if any(type(v) is not int for v in expected.values()):raise ValueError('Integer positions required')
    import scservo_sdk as sdk
    group=json.loads(jc.CONFIG.read_text())['groups']['neck'];port=group['port']
    name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):
                ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            print(json.dumps(control(sdk,ph,pk,group,expected)),flush=True)
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
