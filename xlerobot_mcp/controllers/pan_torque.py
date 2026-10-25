#!/usr/bin/env python3
"""Explicit manual alignment: release or preload-and-hold neck pan ID7 only."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc


def control(sdk,ph,pk,group,action,expected):
    state={k:jc.read(sdk,ph,pk,7,k) for k in jc.REGS}
    limits=group['joints']['7']['observed_raw_limits']
    if not group.get('port_verified') or state['model']!=777 or state['mode']!=0 or state['moving']:
        raise ValueError('Requires verified stationary position-mode neck pan')
    if [state['min'],state['max']]!=limits or not limits[0]<=state['position']<=limits[1]:
        raise ValueError('Calibration changed or pan outside travel limits')
    if abs(state['position']-expected)>8 or not 110<=state['voltage_present_raw']<=130:
        raise ValueError('Position changed or unsuitable voltage')
    tilt_before=jc.read(sdk,ph,pk,8,'torque')
    if jc.read(sdk,ph,pk,8,'moving') or tilt_before!=1:
        raise ValueError('Tilt must be stationary and holding; support camera before releasing pan')
    if state['torque']==1 and abs(state['goal']-state['position'])>8:
        raise ValueError('Outstanding pan target; refusing takeover')
    def torque(value):
        comm,error=pk.write1ByteTxRx(ph,7,40,value)
        if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Pan torque write acknowledgment failed')
        if jc.read(sdk,ph,pk,7,'torque')!=value:raise RuntimeError('Pan torque readback differs')
    if action=='release':
        if state['torque']!=0:torque(0)
    elif action=='hold':
        if state['torque']!=0:raise ValueError('Pan must be released before preload-and-hold')
        start=state['position']
        jc.write(sdk,ph,pk,7,44,0);jc.write(sdk,ph,pk,7,46,100);jc.write(sdk,ph,pk,7,42,start)
        if jc.read(sdk,ph,pk,7,'goal')!=start or abs(jc.read(sdk,ph,pk,7,'position')-start)>4:
            raise RuntimeError('Preload failed or user still moving pan')
        torque(1);time.sleep(.3)
        if abs(jc.read(sdk,ph,pk,7,'position')-start)>8:
            raise RuntimeError('Unexpected displacement after holding enable; inspect before continuing')
    else:raise ValueError('Invalid action')
    tilt_after=jc.read(sdk,ph,pk,8,'torque')
    if tilt_after!=tilt_before:raise RuntimeError('Tilt holding state changed unexpectedly')
    return {'completed':True,'id':7,'action':action,'torque':jc.read(sdk,ph,pk,7,'torque'),
            'position':jc.read(sdk,ph,pk,7,'position'),'tilt_torque_before':tilt_before,'tilt_torque_after':tilt_after}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['release','hold']);p.add_argument('--expected-position',type=int,required=True);a=p.parse_args()
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
            print(json.dumps(control(sdk,ph,pk,group,a.action,a.expected_position)),flush=True)
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
