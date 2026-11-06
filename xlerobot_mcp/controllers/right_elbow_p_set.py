#!/usr/bin/env python3
"""Set right-ID3 position gain to fixed P16 or P32 with torque off; persistent protected setting, no motion."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc
import right_elbow_p_test as pt


def control(sdk,ph,pk,group,p,recovery=pt.RECOVERY):
    if type(p) is not int or p not in (16,32):raise ValueError('Fixed P16 or P32 only')
    if recovery.exists():raise ValueError('Pending P restoration; restore before changing the saved gain')
    if group.get('port_verified') is not True:raise ValueError('Verified right bus required')
    state={k:jc.read(sdk,ph,pk,pt.MOTOR,k) for k in jc.REGS}
    # A gain change with torque on pulls the joint toward its goal; off, nothing can move.
    if state['model']!=777 or state['mode']!=0 or state['moving'] or state['torque']!=0:raise ValueError('ID3 must be stationary, position-mode and torque off; release the supported arm first')
    if not 110<=state['voltage_present_raw']<=130:raise ValueError('Voltage unsuitable for a protected write')
    before={name:pt.read_byte(sdk,ph,pk,addr) for name,addr in [('P',21),('D',22),('I',23),('lock',55)]}
    if before['D']!=32 or before['I']!=0 or before['lock']!=1 or before['P'] not in (16,24,32):raise ValueError('Unexpected ID3 settings; no writes sent')
    if before['P']==p:return {'completed':True,'changed':False,'before':before,'p_raw':p,'lock_raw':before['lock'],'torque':0}
    old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        pt.write_byte(sdk,ph,pk,55,0)
        try:pt.write_byte(sdk,ph,pk,21,p)
        finally:pt.write_byte(sdk,ph,pk,55,1)
    finally:
        for sig,handler in old.items():signal.signal(sig,handler)
    after={'P':pt.read_byte(sdk,ph,pk,21),'lock':pt.read_byte(sdk,ph,pk,55),'torque':jc.read(sdk,ph,pk,pt.MOTOR,'torque')}
    if after!={'P':p,'lock':1,'torque':0}:raise RuntimeError(f'ID3 readback differs: {after}; no retry')
    return {'completed':True,'changed':True,'before':before,'p_raw':p,'lock_raw':1,'torque':0,'note':'No motion commanded. Verify after a power cycle. LeRobot robot.connect() can rewrite P.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--p',type=int,choices=[16,32],required=True);args=parser.parse_args()
    group=json.loads(jc.CONFIG.read_text())['groups']['right-arm']
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            print(json.dumps(control(sdk,ph,pk,group,args.p)),flush=True)
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
