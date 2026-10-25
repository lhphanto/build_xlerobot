#!/usr/bin/env python3
"""One supervised neck commissioning move; explicit session bounds and budget."""
import argparse
import fcntl
import hashlib
import json
import os
import signal
import sys
import tempfile
import time
import joint_control as jc


def validate(group, session, motor, state, target, expected, enable):
    if motor not in (7,8) or group.get('port_verified') is not True:
        raise ValueError('Only verified neck IDs 7 and 8 may be tested')
    if not session.get('enabled') or time.time() >= session.get('expires_unix',0):
        raise ValueError('No active supervised neck test session')
    entry=session['joints'][str(motor)]
    center=entry['center_raw']; radius=entry['radius_raw']
    if type(radius) is not int or not 0 < radius <= 91 or entry['remaining_moves'] <= 0:
        raise ValueError('Invalid test radius or exhausted move budget')
    limits=group['joints'][str(motor)]['observed_raw_limits']
    if [state['min'],state['max']] != limits or not limits[0] <= center-radius < center+radius <= limits[1]:
        raise ValueError('Calibration changed or test window outside travel limits')
    if state['model'] != 777 or state['mode'] != 0 or state['moving']:
        raise ValueError('Requires a stationary STS3215 in position mode')
    if not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
        raise ValueError('Voltage or temperature outside test conditions')
    if abs(state['position']-expected)>8:
        raise ValueError('Position changed since observation')
    if not center-radius <= state['position'] <= center+radius or not center-radius <= target <= center+radius:
        raise ValueError('Current/target outside the supervised test window')
    if not 0 < abs(target-state['position']) <= 99:
        raise ValueError('One small move only; cross-window sweep is not allowed')
    if state['torque'] == 0:
        if not enable or abs(target-state['position']) > 32:
            raise ValueError('Torque-off test needs explicit enable and first step at most 32 ticks')
    elif state['torque'] != 1 or abs(state['goal']-state['position']) > 8:
        raise ValueError('Outstanding target or invalid torque state')
    return entry


def execute(sdk,ph,pk,group,session,motor,target,expected,enable,consume):
    state={key:jc.read(sdk,ph,pk,motor,key) for key in jc.REGS}
    entry=validate(group,session,motor,state,target,expected,enable)
    other=8 if motor==7 else 7
    if jc.read(sdk,ph,pk,other,'moving'):
        raise ValueError('Other neck joint is moving')
    consume()  # persist the attempt before any hardware write; no automatic retry
    start=state['position']; hold_needed=False
    print(json.dumps({'id':motor,'start':start,'target':target,'state':state}),flush=True)
    try:
        if state['torque']==0:
            jc.write(sdk,ph,pk,motor,44,0)
            jc.write(sdk,ph,pk,motor,46,100)
            jc.write(sdk,ph,pk,motor,42,start)
            if jc.read(sdk,ph,pk,motor,'goal')!=start or abs(jc.read(sdk,ph,pk,motor,'position')-start)>4:
                raise RuntimeError('Preload verification failed')
            hold_needed=True
            comm,error=pk.write1ByteTxRx(ph,motor,40,1)
            if comm!=sdk.COMM_SUCCESS or error or jc.read(sdk,ph,pk,motor,'torque')!=1:
                raise RuntimeError('Torque-enable acknowledgment failed')
            time.sleep(.3)
            if abs(jc.read(sdk,ph,pk,motor,'position')-start)>8:
                raise RuntimeError('Unexpected motion while enabling hold')
        hold_needed=True
        reached=jc.nudge(sdk,ph,pk,motor,target)
        final={key:jc.read(sdk,ph,pk,motor,key) for key in jc.REGS}
        if final['moving'] or abs(final['position']-target)>8:
            raise RuntimeError('Endpoint not stable')
        print(json.dumps({'completed':True,'id':motor,'target':target,'reported_position':reached,
                          'final_state':final,'torque':'left enabled'}),flush=True)
        return final
    except BaseException:
        if hold_needed:
            # Repeated interrupts must not interrupt this best-effort hold.
            old={s:signal.signal(s,signal.SIG_IGN) for s in (signal.SIGINT,signal.SIGTERM)}
            try:
                pos=jc.read(sdk,ph,pk,motor,'position')
                lo,hi=group['joints'][str(motor)]['observed_raw_limits']
                if not lo<=pos<=hi:raise RuntimeError('Hold feedback outside travel limits')
                jc.write(sdk,ph,pk,motor,42,pos)
            except Exception as exc:
                print(f'HOLD FAILED: {exc}; use physical support/cutoff if needed.',file=sys.stderr)
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--id',type=int,choices=[7,8],required=True)
    p.add_argument('--target',type=int,required=True)
    p.add_argument('--expected-position',type=int,required=True)
    p.add_argument('--enable-if-off',action='store_true')
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    if not a.execute:
        print(json.dumps({'preview':True,'id':a.id,'target':a.target,'hardware_accessed':False}));return
    import scservo_sdk as sdk
    config=json.loads(jc.CONFIG.read_text());group=config['groups']['neck'];port=group['port']
    name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # Reload session under lock; serializes budget consumption with all controllers.
        config=json.loads(jc.CONFIG.read_text());group=config['groups']['neck'];session=config.get('neck_test_session',{})
        ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):
                ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            def consume():
                session['joints'][str(a.id)]['remaining_moves']-=1
                # Same-directory atomic replacement prevents partial config on interruption.
                fd,tmp=tempfile.mkstemp(dir=jc.CONFIG.parent,prefix='.neck-config-')
                try:
                    with os.fdopen(fd,'w') as out:
                        json.dump(config,out,indent=2);out.write('\n');out.flush();os.fsync(out.fileno())
                    os.replace(tmp,jc.CONFIG)
                finally:
                    if os.path.exists(tmp):os.unlink(tmp)
            execute(sdk,ph,pk,group,session,a.id,a.target,a.expected_position,a.enable_if_off,consume)
        finally:ph.closePort()

if __name__=='__main__':
    def interrupted(signum,frame):raise KeyboardInterrupt('Neck test interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    try:main()
    except (Exception,KeyboardInterrupt) as exc:
        print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
