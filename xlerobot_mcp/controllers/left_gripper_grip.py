#!/usr/bin/env python3
"""Supervised left gripper (ID6): open to a position, or close until contact and hold a bounded squeeze. No other joint writes."""
import argparse,fcntl,hashlib,json,os,signal,sys,tempfile,time
import joint_control as jc
import left_arm_hold as ah
import left_elbow_test as et

GRIPPER='6'
MAX_OPEN=700          # ticks per open command (~61 degrees); positive ticks open
STEP=16               # closing step
SPEED=30
REACHED=4             # a step counts as followed within this many ticks
TICKS_PER_S=50        # measured free opening rate at speed 30 (November16)
CONTACT_LOAD=300      # masked present load that means something is resisting
MAX_SQUEEZE=24        # past contact; force comes from P x this offset
MAX_TIGHTEN=4         # one direct adjustment after visually confirmed contact; no search
TIGHTEN_LOAD=150      # raw load guard, NOT calibrated force
FLOOR_MARGIN=8        # never command closer than this to the closed limit
REACH_GOAL_TOLERANCE={'1':16,'2':40,'3':24,'4':16,'5':16,'6':MAX_SQUEEZE}  # as left_reach.GOAL_TOLERANCE (literal: no placo import); ID6 may hold a squeeze (open/re-close after a grip)


def packet(sdk,ph,pk,target):
    comm,error=pk.writeTxRx(ph,6,42,6,[target & 255,target >> 8,0,0,SPEED,0])
    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Gripper target acknowledgment failed; no retry')
    if jc.read(sdk,ph,pk,6,'goal')!=target:raise RuntimeError('Gripper target readback mismatch')


def load(sdk,ph,pk):
    addr,size=jc.STATUS_REGS['present_load_raw']
    value,comm,error=pk.read2ByteTxRx(ph,6,addr)
    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Gripper load read failed')
    return value & 1023  # bit 10 is direction


def control(sdk,ph,pk,group,expected,action,target=None,squeeze=12):
    if action not in ('open','close','tighten'):raise ValueError('Action must be open, close or tighten')
    initial=ah.snapshot(sdk,ph,pk)
    et.validate(group,initial,expected,0,REACH_GOAL_TOLERANCE)  # all six holding and stationary; a completed reach may leave these offsets
    lo,hi=group['joints'][GRIPPER]['safe_raw_limits']
    start=initial[GRIPPER]['position'];floor=lo+FLOOR_MARGIN
    if action=='open':
        if type(target) is not int or not start<target<=min(hi,start+MAX_OPEN):raise ValueError(f'Open target must be above {start} and at most {min(hi,start+MAX_OPEN)}')
    elif action=='tighten':
        if type(squeeze) is not int or not 1<=squeeze<=MAX_TIGHTEN:raise ValueError('Tighten must be 1-4 ticks')
        if start-squeeze<floor:raise ValueError('Tighten target reaches the closed floor')
        if abs(initial[GRIPPER]['goal']-start)>2:raise ValueError('Gripper already has a holding offset; inspect before tightening')
        if start-squeeze>=initial[GRIPPER]['goal']:raise ValueError('Adjustment would not tighten beyond the existing target')
        if load(sdk,ph,pk)>=TIGHTEN_LOAD:raise ValueError('Gripper already loaded; do not add squeeze')
    elif type(squeeze) is not int or not 0<=squeeze<=MAX_SQUEEZE:raise ValueError(f'Squeeze must be 0-{MAX_SQUEEZE} ticks')
    trace=[];glitches=[];commanded=initial[GRIPPER]['goal'];attempted=False;began=time.monotonic()

    def monitor():
        states=ah.snapshot(sdk,ph,pk)
        for mid,s in states.items():
            if not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:
                # One corrupt sample read ID1 at 77 C (29-31 C before and after, November16): confirm before stopping.
                v,t=jc.read(sdk,ph,pk,int(mid),'voltage_present_raw'),jc.read(sdk,ph,pk,int(mid),'temperature_raw')
                if not 110<=v<=130 or t>=60:raise RuntimeError(f"Power or temperature unsuitable: ID{mid} voltage {s['voltage_present_raw']}/{v}, temperature {s['temperature_raw']}/{t}")
                glitches.append({'id':mid,'voltage':s['voltage_present_raw'],'temperature':s['temperature_raw'],'reread':[v,t]})
            if s['model']!=777 or s['mode']!=0 or s['torque']!=1:raise RuntimeError(f'ID{mid} holding state changed')
            if mid==GRIPPER:
                if s['goal']!=commanded or not lo<=s['position']<=hi:raise RuntimeError('Gripper target or position out of bounds')
            elif abs(s['position']-initial[mid]['position'])>8 or s['goal']!=initial[mid]['goal']:raise RuntimeError(f'ID{mid} moved during a gripper command')
        row={'t':round(time.monotonic()-began,3),'goal':commanded,'position':states[GRIPPER]['position'],'moving':states[GRIPPER]['moving'],'load':load(sdk,ph,pk)}
        trace.append(row)
        if len(trace)>160:del trace[80]  # keep the first 80 and the latest 80 samples (contact happens at the end)
        return row

    commanded_from=initial[GRIPPER]['position']
    def send(goal):
        nonlocal commanded,attempted,commanded_from
        commanded_from=jc.read(sdk,ph,pk,6,'position');attempted=True;commanded=goal;packet(sdk,ph,pk,goal)

    def settle(goal,seconds=None):
        """Wait until the gripper is still; True if it reached goal within REACHED ticks."""
        if seconds is None:seconds=2+abs(goal-commanded_from)/TICKS_PER_S*1.25
        last=None;still=0;deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            row=monitor()
            still=still+1 if last is not None and abs(row['position']-last)<=1 and not row['moving'] else 0
            last=row['position']
            if abs(row['position']-goal)<=REACHED and still>=2:return True,row
            if still>=3:return False,row  # stopped short: blocked
            time.sleep(.05)
        raise TimeoutError('Gripper did not settle in time')

    try:
        if action=='tighten':
            # Exactly one small target. Never run the16-tick contact search.
            goal=start-squeeze;send(goal);deadline=time.monotonic()+2;last=None;still=0
            while time.monotonic()<deadline:
                row=monitor()
                if row['load']>=TIGHTEN_LOAD:raise RuntimeError('Tighten load guard reached; force is uncalibrated')
                if not goal-2<=row['position']<=start+2:raise RuntimeError('Gripper moved outside the small adjustment window')
                still=still+1 if last is not None and abs(row['position']-last)<=1 and not row['moving'] else 0
                last=row['position']
                if still>=3:
                    return {'completed':True,'action':'tighten','start_position':start,'hold_goal':goal,
                            'requested_ticks':squeeze,'holding_position':row['position'],'holding_load':row['load'],
                            'note':'One bounded target offset, not millimeters or measured force; grip security unverified.',
                            'trace':trace,'glitches':glitches}
                time.sleep(.05)
            raise TimeoutError('Small gripper adjustment did not become stationary')
        if action=='open':
            send(target);ok,row=settle(target)
            if not ok:raise RuntimeError(f'Gripper stopped at {row["position"]} opening toward {target}; something is in the way')
            return {'completed':True,'action':'open','position':row['position'],'opened_ticks':row['position']-start,'trace':trace,'glitches':glitches}
        position=start;contact=None
        while position>floor:
            goal=max(floor,position-STEP);send(goal)
            ok,row=settle(goal)
            if not ok or row['load']>=CONTACT_LOAD:
                contact=row;break
            position=row['position']
            # A reached floor may still have up to REACHED ticks of servo deadband.
            # Do not resend it forever while waiting for exact encoder equality.
            if goal==floor:break
        if contact is None:
            return {'completed':True,'action':'close','contact':False,'position':position,'note':'Closed to the floor without contact: nothing gripped.','trace':trace,'glitches':glitches}
        hold=max(floor,contact['position']-squeeze);send(hold)
        time.sleep(.3);row=monitor()
        return {'completed':True,'action':'close','contact':True,'contact_position':contact['position'],'contact_load':contact['load'],
                'hold_goal':hold,'squeeze_ticks':contact['position']-hold,'holding_position':row['position'],'holding_load':row['load'],
                'note':'Grip force comes from the squeeze offset; release by opening.','trace':trace,'glitches':glitches}
    except BaseException as exc:
        cleanup={'attempted':False}
        if attempted:
            old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                pos=jc.read(sdk,ph,pk,6,'position')
                if lo<=pos<=hi:packet(sdk,ph,pk,pos);cleanup={'attempted':True,'hold_goal':pos}
            except Exception as error:cleanup={'attempted':True,'error':str(error)}
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
        return {'completed':False,'action':action,'error':str(exc) or type(exc).__name__,'cleanup':cleanup,'trace':trace,'glitches':glitches}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['open','close','tighten'])
    parser.add_argument('--expected-positions',required=True);parser.add_argument('--target',type=int);parser.add_argument('--squeeze',type=int,default=12)
    args=parser.parse_args();expected=json.loads(args.expected_positions)
    if set(expected)!=set(map(str,ah.IDS)) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):raise ValueError('Six integer observed positions required')
    group=json.loads(jc.CONFIG.read_text())['groups']['left-arm']
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            result=control(sdk,ph,pk,group,expected,args.action,args.target,args.squeeze)
            print(json.dumps(result),flush=True);return 0 if result['completed'] else 1
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
