#!/usr/bin/env python3
"""One supervised left shoulder +32tick slow lift and hold; no return."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import right_arm_test as core


def control(sdk,ph,pk,group,expected):
    result=core.control(sdk,ph,pk,group,expected,2,return_to_start=False,speed_raw=30)
    result['note']='Left shoulder lift upward verified by operator. Single +32tick joint-space lift; gripper path is not calibrated. No other joint writes or automatic return. Holding torque remains on.'
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--expected-positions',required=True);a=p.parse_args()
    group=json.loads(core.jc.CONFIG.read_text())['groups']['left-arm'];expected=json.loads(a.expected_positions)
    if set(expected)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):raise ValueError('All6 fresh integer positions required')
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            print(json.dumps(control(sdk,ph,pk,group,expected)),flush=True)
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
