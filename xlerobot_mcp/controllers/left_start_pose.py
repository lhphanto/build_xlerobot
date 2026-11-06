#!/usr/bin/env python3
"""Preview or execute one supervised arm move to its fixed start pose; IDs2-4 only, then hold."""
import argparse,fcntl,hashlib,json,math,os,signal,sys,tempfile,time
import joint_control as jc
import left_arm_hold as ah
import left_elbow_test as et
import right_gripper_up as reach
from settle_timeout import stationary_timeout_evidence

# Inside the shoulder840-870 / wrist2140-2160 area where a20mm coarse lift plans at elbow2787.
START={'2':850,'3':2787,'4':2150}
MAX_TRAVEL,STAGE_TICKS=128,16
FINAL_TOLERANCE=8  # Keeps the shoulder and wrist inside the lift start area.
TRIM_ABOVE=2
LEFT={'group':'left-arm','policy':'left_start_pose_v1','start':START,
      'stage_tolerance':{'2':8,'3':16,'4':8},  # Waypoint arrival; elbow sag is the largest.
      'max_goal_offset':{'2':8,'3':16,'4':8},  # Stay within the lift preflight's goal-error bound.
      'calibration':lambda group:group['kinematic_calibration']}
MAX_HEIGHT_M,MAX_SIDE_M=.07,.02
LIVE_MARGIN_M=.01


def make_plan(group,states,expected,observer,arm=LEFT):
    """No hardware writes. Waypoints are desired positions from the measured pose to the arm's start."""
    START=arm['start']
    et.validate(group,states,expected,0)  # All six holding and stationary, same preflight as the elbow tests.
    start={m:states[m]['position'] for m in START}
    for mid,target in START.items():
        lo,hi=group['joints'][mid]['safe_raw_limits']
        if not lo<=target<=hi:raise ValueError(f'Start pose ID{mid} outside saved limits')
        if abs(target-start[mid])>MAX_TRAVEL:raise ValueError(f'ID{mid} is {abs(target-start[mid])} ticks from the start pose; above {MAX_TRAVEL}, reposition by hand with supported release')
    largest=max(abs(START[m]-start[m]) for m in START)
    steps=math.ceil(largest/STAGE_TICKS)
    waypoints=[] if largest<=TRIM_ABOVE else [{m:int(round(start[m]+(START[m]-start[m])*step/steps)) for m in START} for step in range(1,steps+1)]
    modeled=[]
    for point in waypoints:
        delta=observer({m:{'position':point.get(m,states[m]['position'])} for m in states})
        side=math.hypot(delta[0],delta[1])
        if abs(delta[2])>MAX_HEIGHT_M or side>MAX_SIDE_M:raise ValueError('Modeled gripper travel exceeds the start-pose envelope')
        modeled.append({'height_m':delta[2],'side_m':side})
    identity={'policy':arm['policy'],'expected':expected,'targets':START,'waypoints':waypoints,'goals':{m:s['goal'] for m,s in states.items()},
              'limits':{m:group['joints'][m]['safe_raw_limits'] for m in sorted(group['joints'])}}
    return {'execution_policy':arm['policy'],'expected_positions':expected,'targets':START.copy(),'waypoints':waypoints,'modeled_gripper':modeled,
            'final_modeled_height_change_mm':modeled[-1]['height_m']*1000 if modeled else 0.,'already_at_start':not waypoints,
            'plan_hash':hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest(),
            'note':'Height is computed from the uncalibrated SO101 model. Negative means the gripper moves down; keep that space clear.'}


def execute(sdk,ph,pk,group,initial,p,observer,arm=LEFT):
    START=arm['start']
    if p.get('execution_policy')!=arm['policy'] or p.get('targets')!=START:raise ValueError('Unsupported start-pose plan; obtain a new preview')
    commands={m:s['goal'] for m,s in initial.items()};trace=[];recent={m:[] for m in START};attempted=set();cleanup={'attempted':False}
    stage=0
    def write(mid,target):
        attempted.add(mid);commands[mid]=target
        reach.packet(sdk,ph,pk,int(mid),target)
    def monitor():
        states=ah.snapshot(sdk,ph,pk)
        for mid,s in states.items():
            baseline=initial[mid];lo,hi=group['joints'][mid]['safe_raw_limits']
            if not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:raise RuntimeError(f"Power or temperature unsuitable: ID{mid} voltage {s['voltage_present_raw']}, temperature {s['temperature_raw']}")
            if s['model']!=777 or s['mode']!=0 or s['torque']!=1 or [s['min'],s['max']]!=[baseline['min'],baseline['max']] or not lo<=s['position']<=hi or not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60 or s['goal']!=commands[mid]:raise RuntimeError('Joint target/state/power changed during start-pose move')
            if mid in START:
                a,b=p['expected_positions'][mid],START[mid]
                if not min(a,b)-16<=s['position']<=max(a,b)+16:raise RuntimeError(f'ID{mid} escaped the start-pose window')
                row={'elapsed_s':time.monotonic(),'stage':stage,'target_raw':commands[mid],'goal_raw':s['goal'],'position_raw':s['position'],'moving':s['moving'],'torque':s['torque'],'voltage_raw':s['voltage_present_raw'],'temperature_c':s['temperature_raw']}
                for name in ('present_speed_raw','present_load_raw','servo_status_raw','motor_phase_current_raw'):
                    addr,size=jc.STATUS_REGS[name];row[name]=reach.read_extra(sdk,ph,pk,int(mid),addr,size)
                if row['servo_status_raw']:raise RuntimeError('Servo status fault during start-pose move')
                recent[mid].append(row);del recent[mid][:-3]
            elif abs(s['position']-baseline['position'])>8 or s['moving']:raise RuntimeError('An untouched joint moved')
        delta=observer(states)
        if len(trace)<128:trace.append({'stage':stage,'positions':{m:states[m]['position'] for m in START},'goals':{m:commands[m] for m in START},'modeled_height_m':delta[2],'min_voltage_raw':min(s['voltage_present_raw'] for s in states.values())})
        if abs(delta[2])>MAX_HEIGHT_M+LIVE_MARGIN_M or math.hypot(delta[0],delta[1])>MAX_SIDE_M+LIVE_MARGIN_M:raise RuntimeError('Modeled gripper left the start-pose envelope')
        return states
    def settle():
        # Sampled stillness as well as the flag: a sagging joint can keep creeping.
        deadline=time.monotonic()+3;stable=0;last=None
        while time.monotonic()<deadline:
            states=monitor();now={m:states[m]['position'] for m in START}
            still=last is not None and all(abs(now[m]-last[m])<=1 and not states[m]['moving'] for m in START)
            stable=stable+1 if still else 0;last=now
            if stable>=3:return states
            time.sleep(.05)
        raise TimeoutError('Joints did not become stationary in 3 seconds')
    try:
        for stage,point in enumerate(p['waypoints'],1):
            monitor()
            for mid in START:
                if point[mid]!=commands[mid]:write(mid,point[mid])
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                states=monitor()
                if all(abs(states[m]['position']-point[m])<=arm['stage_tolerance'][m] for m in START):break
                time.sleep(.05)
            else:
                raise TimeoutError(f'Start-pose stage{stage} stopped: position-minus-waypoint ticks={ {m:states[m]["position"]-point[m] for m in START} }. No next waypoint attempted')
        stage=len(p['waypoints'])+1
        states=settle();trimmed={}
        # One feedback correction only: gravity moves the landing point by a pose-dependent amount.
        for mid,target in START.items():
            residual=states[mid]['position']-target
            if abs(residual)>TRIM_ABOVE:
                lo,hi=group['joints'][mid]['safe_raw_limits']
                command=max(lo,min(hi,max(target-arm['max_goal_offset'][mid],min(target+arm['max_goal_offset'][mid],commands[mid]-residual))))
                if command!=commands[mid]:write(mid,command);trimmed[mid]=command
        if trimmed:stage+=1;states=settle()
        errors={m:states[m]['position']-START[m] for m in START}
        return {'completed':all(abs(v)<=FINAL_TOLERANCE for v in errors.values()),'final_errors_ticks':errors,'final_tolerance_ticks':FINAL_TOLERANCE,'trimmed_commands':trimmed,'plan':p,'states':states,'trace':trace,'torque':'left on',
                'note':'A miss is reported, not retried. Preview a lift next; the planner decides whether this pose is usable. Operator confirms clearance.'}
    except BaseException as exc:
        evidence={m:stationary_timeout_evidence(recent[m],commands[m],stage) for m in attempted}
        if isinstance(exc,TimeoutError) and attempted and all(evidence.values()):
            cleanup.update({'action':'preserve_existing_targets','target_write_sent':False,'verified_stationary':evidence})
        elif attempted:
            cleanup['attempted']=True;cleanup['holds']={}
            old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                for mid in sorted(attempted):
                    try:
                        if jc.read(sdk,ph,pk,int(mid),'torque')!=1:raise RuntimeError('Torque not on; not re-enabling')
                        pos=jc.read(sdk,ph,pk,int(mid),'position');lo,hi=group['joints'][mid]['safe_raw_limits'];a,b=p['expected_positions'][mid],START[mid]
                        if not lo<=pos<=hi or not min(a,b)-16<=pos<=max(a,b)+16:raise RuntimeError('Hold feedback outside the start-pose window')
                        reach.packet(sdk,ph,pk,int(mid),pos);cleanup['holds'][mid]={'acknowledged':True,'target':pos}
                    except Exception as error:cleanup['holds'][mid]={'acknowledged':False,'error':str(error)}
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
        return {'completed':False,'error':str(exc) or type(exc).__name__,'plan':p,'trace':trace,'cleanup':cleanup,'note':'No automatic return, retry or torque release; keep arm supported.'}


def main(arm=LEFT):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['preview','execute']);parser.add_argument('--expected-positions',required=True);parser.add_argument('--plan-hash');args=parser.parse_args()
    expected=json.loads(args.expected_positions)
    if set(expected)!=set(map(str,ah.IDS)) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):raise ValueError('Six integer observed positions required')
    group=json.loads(jc.CONFIG.read_text())['groups'][arm['group']]
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            observer=reach.make_observer(expected,arm['calibration'](group))
            states=ah.snapshot(sdk,ph,pk);p=make_plan(group,states,expected,observer,arm)
            if args.action=='preview':result={'completed':True,'plan':p,'hardware_writes':False}
            else:
                if args.plan_hash!=p['plan_hash']:raise ValueError('Preview identity changed; obtain a new preview')
                result=execute(sdk,ph,pk,group,states,p,observer,arm)
            print(json.dumps(result),flush=True);return 0 if result['completed'] else 1
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
