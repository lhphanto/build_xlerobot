#!/usr/bin/env python3
"""One supported right-ID3 +32tick trial at P16 or P24; restore original P16."""
import argparse
import contextlib
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import right_arm_test as rt

ROOT = Path(__file__).resolve().parents[1]
RECOVERY = ROOT/'.private/right-elbow-p-recovery.json'
MOTOR = 3


def read_byte(sdk, ph, pk, addr):
    value, comm, error = pk.read1ByteTxRx(ph, MOTOR, addr)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'ID3 read failed at {addr}')
    return value


def write_byte(sdk, ph, pk, addr, value):
    if (addr, value) not in ((21,16),(21,24),(55,0),(55,1)):
        raise ValueError('Only fixed ID3 P and lock writes allowed')
    comm, error = pk.write1ByteTxRx(ph, MOTOR, addr, value)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'ID3 write acknowledgment failed at {addr}; no retry')
    if read_byte(sdk, ph, pk, addr) != value:
        raise RuntimeError(f'ID3 readback mismatch at {addr}')


def write_record(path, port):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Reserved before any protected write. Survives server/process restarts.
    with path.open('x') as handle:
        os.chmod(path,0o600)
        json.dump({'port':port,'id':3,'original_p':16,'original_lock':1,
                   'trial_p':24,'created_unix':time.time()},handle)
        handle.flush()
        os.fsync(handle.fileno())
    fd=os.open(str(path.parent),os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)


def restore(sdk, ph, pk, port, path):
    record=json.loads(path.read_text())
    if any(record.get(k)!=v for k,v in {'port':port,'id':3,'original_p':16,'original_lock':1,'trial_p':24}.items()):
        raise ValueError('Recovery record does not match fixed right-ID3 settings')
    failures=[]
    # Readback is required even after a lost acknowledgment. No motion retry.
    try:
        if read_byte(sdk,ph,pk,21)!=16:
            try:write_byte(sdk,ph,pk,55,0)
            except Exception as exc:failures.append(str(exc))
            # Only write P when unlock is positively verified.
            if read_byte(sdk,ph,pk,55)==0:
                try:write_byte(sdk,ph,pk,21,16)
                except Exception as exc:failures.append(str(exc))
    except Exception as exc:failures.append(str(exc))
    finally:
        try:write_byte(sdk,ph,pk,55,1)
        except Exception as exc:failures.append(str(exc))
    try:verified=read_byte(sdk,ph,pk,21)==16 and read_byte(sdk,ph,pk,55)==1
    except Exception as exc:failures.append(str(exc));verified=False
    if verified:path.unlink()
    return {'verified':verified,'p_raw':16 if verified else None,'lock_raw':1 if verified else None,
            'errors':failures,'recovery_pending':not verified}


def control(sdk, ph, pk, group, expected, trial_p, path=RECOVERY, action='trial'):
    if type(trial_p) is not int or trial_p not in (16,24) or action not in ('trial','restore'):
        raise ValueError('Fixed P16/P24 trial or recovery only')
    if action=='restore':
        if not path.exists():raise ValueError('No pending P recovery')
        # Recovery does not command motion/torque and cannot select a free register.
        state=rt.ah.snapshot(sdk,ph,pk)
        if group.get('port_verified') is not True or state['3']['model']!=777 or state['3']['mode']!=0:
            raise ValueError('Recovery requires verified position-mode right ID3')
        old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:result=restore(sdk,ph,pk,group['port'],path)
        finally:
            for sig,handler in old.items():signal.signal(sig,handler)
        return {'completed':result['verified'],'action':'restore','restoration':result}
    if path.exists():raise ValueError('Pending P restoration blocks new tuning trials; restore first')
    prepared=rt.prepare(group,expected,3)
    initial=rt.ah.snapshot(sdk,ph,pk)
    start=rt.validate(prepared,initial,expected,3,[32,0])
    # Gain changes must not amplify an existing target error.
    if abs(initial['3']['goal']-start)>4:
        raise ValueError('ID3 requires holding goal within4ticks before a gain change')
    settings={name:read_byte(sdk,ph,pk,addr) for name,addr in [('P',21),('D',22),('I',23),('lock',55)]}
    if settings!={'P':16,'D':32,'I':0,'lock':1}:
        raise ValueError('Expected original ID3 P16/D32/I0 with lock1; no writes sent')
    result={'completed':False,'action':'trial','trial_p':trial_p,'original_settings':settings,
            'start_raw':start,'restoration':{'verified':True,'writes_needed':False}}
    protected_attempt=False
    try:
        if trial_p==24:
            write_record(path,group['port'])
            protected_attempt=True
            write_byte(sdk,ph,pk,55,0)
            try:write_byte(sdk,ph,pk,21,24)
            finally:write_byte(sdk,ph,pk,55,1)
        # Check P, pose and all six joints again after applying the gain.
        if read_byte(sdk,ph,pk,21)!=trial_p:raise RuntimeError('Trial P not verified')
        now=rt.ah.snapshot(sdk,ph,pk)
        rt.validate(prepared,now,expected,3,[32,0])
        if abs(now['3']['position']-start)>4:raise RuntimeError('Pose changed after P update')
        captured=io.StringIO()
        try:
            with contextlib.redirect_stdout(captured):
                result['trial']=rt.control(sdk,ph,pk,group,expected,3)
            result['completed']=True
        except BaseException:
            lines=captured.getvalue().splitlines()
            if lines:
                try:result['trial_failure']=json.loads(lines[-1])
                except json.JSONDecodeError:result['trial_output']=captured.getvalue()[-4000:]
            raise
    except BaseException as exc:
        result['error']=str(exc) or type(exc).__name__
    finally:
        if protected_attempt:
            old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                try:result['restoration']=restore(sdk,ph,pk,group['port'],path)
                except Exception as exc:result['restoration']={'verified':False,'error':str(exc),'recovery_pending':True}
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
            if not result['restoration']['verified']:result['completed']=False
    result['recovery_pending']=path.exists()
    result['note']='No automatic torque release. Keep arm supported. A failed trial is not accuracy-qualified.'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--p',type=int,choices=[16,24],default=16)
    parser.add_argument('--action',choices=['trial','restore'],default='trial')
    parser.add_argument('--expected-positions',default='{}')
    args=parser.parse_args()
    group=json.loads(rt.jc.CONFIG.read_text())['groups']['right-arm']
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            result=control(sdk,ph,pk,group,json.loads(args.expected_positions),args.p,action=args.action)
            print(json.dumps(result),flush=True)
            return 0 if result['completed'] else 1
        finally:ph.closePort()

if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
