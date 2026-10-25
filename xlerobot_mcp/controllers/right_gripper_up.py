#!/usr/bin/env python3
"""Preview or execute one fixed2mm +arm-model-Z reach, IDs2-4 only."""
import argparse,contextlib,fcntl,hashlib,itertools,json,math,os,signal,sys,tempfile,time
from pathlib import Path
import right_arm_test as rt
from settle_timeout import stationary_timeout_evidence
ROOT=Path(__file__).resolve().parents[1]
URDF=ROOT.parent/'SO101/so101_new_calib.urdf'
CAL=Path('/Users/huiliu/.cache/huggingface/lerobot/calibration/robots/so_follower/lxh_right_arm.json')
NAMES=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
ACTIVE=(2,3,4)


def identity(group,expected,cal_bytes,urdf_bytes,step_mm=2):
    payload={'group':group,'expected':expected,'cal':hashlib.sha256(cal_bytes).hexdigest(),'urdf':hashlib.sha256(urdf_bytes).hexdigest(),'step_mm':step_mm}
    return hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()


def read_extra(sdk,ph,pk,motor,addr,size=1):
    fn=pk.read2ByteTxRx if size==2 else pk.read1ByteTxRx
    value,comm,error=fn(ph,motor,addr)
    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} diagnostic/calibration read failed at{addr}')
    return value


def preflight(group,states,expected,cal,offsets,execute=False,goal_error_limit=4,coarse=False):
    if goal_error_limit not in (4,8):raise ValueError('Invalid startup holding-error policy')
    if group.get('port_verified') is not True or set(expected)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):raise ValueError('Verified right bus and all6 fresh integer positions required')
    for motor,name in enumerate(NAMES,1):
        mid=str(motor);s=states[mid];j=group['joints'][mid];c=cal[name];limits=j['safe_raw_limits']
        if not isinstance(limits,list) or len(limits)!=2:raise ValueError('Approved software limits required for each joint')
        lo,hi=limits
        if c['id']!=motor or c['drive_mode']!=0 or [c['range_min'],c['range_max']]!=[s['min'],s['max']] or [s['min'],s['max']]!=j['observed_raw_limits'] or not s['min']<=lo<hi<=s['max']:raise ValueError('Calibration/range/mapping mismatch')
        if offsets[mid]!=c['homing_offset']:raise ValueError('Saved calibration homing offset is stale')
        if s['model']!=777 or s['mode']!=0 or s['moving'] or s['torque'] not in (0,1):raise ValueError('Stationary position-mode STS3215 required')
        if execute and (s['torque']!=1 or abs(s['goal']-s['position'])>(16 if coarse and motor==3 else goal_error_limit)):raise ValueError('All6 must already hold within the startup goal-error bound')
        if not lo<=s['position']<=hi or abs(s['position']-expected[mid])>4:raise ValueError('Pose changed or outside saved limits')
        if not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60:raise ValueError('Power or temperature unsuitable')


@contextlib.contextmanager
def solver_stdout_to_stderr():
    # placo can print native C++ collision notices; keep tool stdout valid JSON.
    sys.stdout.flush();saved=os.dup(1)
    try:os.dup2(2,1);yield
    finally:os.dup2(saved,1);os.close(saved)


def solve(expected,cal,urdf=URDF,step_mm=2,orientation_weight=.01,lag_guard=False,coarse=False):
    if orientation_weight not in (0,.01) or type(lag_guard) is not bool:raise ValueError('Fixed supported IK policy required')
    if type(step_mm) not in (int,float) or not math.isfinite(step_mm) or not 2<=step_mm<=(20 if coarse else 10):raise ValueError('Distance must be finite and between2and10mm')
    if type(coarse) is not bool:raise ValueError('coarse must be boolean')
    height=step_mm/1000
    import numpy as np
    import xml.etree.ElementTree as ET
    from lerobot.model.kinematics import RobotKinematics
    names=NAMES[:5]
    mid=lambda name:(cal[name]['range_min']+cal[name]['range_max'])/2
    q=np.array([(expected[str(cal[n]['id'])]-mid(n))*360/4095 for n in names])
    with solver_stdout_to_stderr():
        k=RobotKinematics(str(urdf),joint_names=list(names))
        origin=k.forward_kinematics(q).copy();wanted=origin.copy();wanted[2,3]+=height
        result=k.inverse_kinematics(q,wanted,orientation_weight=orientation_weight,max_iters=32)
    if not np.isfinite(result).all():raise ValueError('Non-finite IK result')
    target={str(i):expected[str(i)] for i in range(1,7)}
    for i in ACTIVE:target[str(i)]=int(round(result[i-1]*4095/360+mid(names[i-1])))
    limits={j.attrib['name']:[float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper'])] for j in ET.parse(urdf).getroot().findall('joint') if j.find('limit') is not None}
    def model(raw):
        values=np.array([(raw[str(cal[n]['id'])]-mid(n))*360/4095 for n in names])
        if any(not limits[n][0]<=np.deg2rad(values[i])<=limits[n][1] for i,n in enumerate(names)):raise ValueError('Start/path/target outside URDF limits')
        return k.forward_kinematics(values).copy()
    model(expected)
    steps=max(1,math.ceil(max(abs(target[str(i)]-expected[str(i)]) for i in ACTIVE)/8))
    if steps>(8 if coarse else 4):raise ValueError('IK requires more than32ticks per joint; no movement')
    waypoints=[];prior=expected.copy();last_height=0.;worst_partial_height=0.
    for step in range(1,steps+1):
        raw=expected.copy()
        for motor in ACTIVE:raw[str(motor)]=int(round(expected[str(motor)]+(target[str(motor)]-expected[str(motor)])*step/steps))
        t=model(raw);d=t[:3,3]-origin[:3,3]
        angle=float(np.rad2deg(np.arccos(np.clip((np.trace(origin[:3,:3].T@t[:3,:3])-1)/2,-1,1))))
        if d[2]<last_height-.0002 or not -.0002<=d[2]<=height+.001 or np.linalg.norm(d[:2])>.001 or angle>(6 if coarse else 3):raise ValueError('Model path not a small upward reach')
        if any(abs(raw[str(i)]-prior[str(i)])>8 for i in ACTIVE):raise ValueError('Waypoint joint increment exceeds8ticks')
        if lag_guard:
            # Independent response delays plus existing4tick waypoint tolerance.
            # Each joint may lag toward its last tolerated achieved position.
            lag_start=expected.copy()
            for motor in ACTIVE:
                mid_id=str(motor);travel=prior[mid_id]-expected[mid_id]
                lag_start[mid_id]=expected[mid_id]+int(math.copysign(max(0,abs(travel)-4),travel)) if travel else expected[mid_id]
            for fractions in itertools.product((0,.5,1),repeat=3):
                partial=expected.copy()
                for motor,fraction in zip(ACTIVE,fractions):
                    mid_id=str(motor);partial[mid_id]=int(round(lag_start[mid_id]+fraction*(raw[mid_id]-lag_start[mid_id])))
                partial_delta=model(partial)[:3,3]-origin[:3,3]
                worst_partial_height=min(worst_partial_height,float(partial_delta[2]))
                if partial_delta[2]<-.0005 or np.linalg.norm(partial_delta[:2])>.0015:raise ValueError('Plan unsafe under bounded independent joint lag')
        waypoints.append({str(i):raw[str(i)] for i in ACTIVE});prior=raw;last_height=float(d[2])
    endpoint=model(target);residual=float(np.linalg.norm(endpoint[:3,3]-wanted[:3,3]))
    if residual>.0005 or last_height<height-.0005:raise ValueError('Rounded model reach does not meet requested height within0.5mm')
    return {'targets':{str(i):target[str(i)] for i in ACTIVE},'waypoints':waypoints,
            'predicted_displacement_m':(endpoint[:3,3]-origin[:3,3]).tolist(),'model_residual_m':residual,'orientation_weight':orientation_weight,'modeled_orientation_change_deg':angle,'lag_guard':lag_guard,'worst_modeled_partial_height_m':worst_partial_height}


def plan(group,states,expected,cal,offsets,cal_bytes,urdf_bytes,execute=False,solver=solve,step_mm=2,goal_error_limit=4,waypoint_limit=8,coarse=False):
    if type(step_mm) not in (int,float) or not math.isfinite(step_mm) or not 2<=step_mm<=(20 if coarse else 10):raise ValueError('Distance must be finite and between2and10mm')
    if type(coarse) is not bool:raise ValueError('coarse must be boolean')
    if waypoint_limit not in (8,16):raise ValueError('Unsupported waypoint bound')
    preflight(group,states,expected,cal,offsets,execute,goal_error_limit,coarse)
    result=solver(expected,cal)
    if set(result['targets'])!=set(map(str,ACTIVE)) or not 1<=len(result['waypoints'])<=(6 if coarse else 4):raise ValueError('Invalid fixed-joint plan')
    prior={str(i):expected[str(i)] for i in ACTIVE}
    for point in result['waypoints']:
        if set(point)!=set(map(str,ACTIVE)):raise ValueError('Plan addresses other joints')
        for mid,target in point.items():
            lo,hi=group['joints'][mid]['safe_raw_limits']
            if type(target) is not int or not lo<=target<=hi or abs(target-expected[mid])>(64 if coarse else 32) or abs(target-states[mid]['position'])>(64 if coarse else 32) or abs(target-prior[mid])>waypoint_limit:raise ValueError('Waypoint violates saved or incremental limits')
        prior=point
    if prior!=result['targets']:raise ValueError('Final target mismatch')
    inputs_hash=identity(group,expected,cal_bytes,urdf_bytes,step_mm)
    plan_hash=hashlib.sha256(json.dumps({'inputs_hash':inputs_hash,'targets':result['targets'],'waypoints':result['waypoints'],'execution_policy':result.get('execution_policy'),'stage_minimum_heights_m':result.get('stage_minimum_heights_m'),'waypoint_limit':waypoint_limit,'coarse':coarse,'joint_tolerances':result.get('joint_tolerances')},sort_keys=True).encode()).hexdigest()
    return {**result,'plan_hash':plan_hash,'expected_positions':expected,
            'step_mm':step_mm,'frame':'standalone SO101 model +Z; physical direction/distance require operator observation',
            'model_to_body_verified':False,'preview_only':not execute}


def calibration_offsets(sdk,ph,pk):
    result={}
    for motor in range(1,7):
        raw=read_extra(sdk,ph,pk,motor,31,2)
        result[str(motor)]=-(raw & 2047) if raw & 2048 else raw
        if read_extra(sdk,ph,pk,motor,30)!=1:raise ValueError('Unexpected encoder resolution')
    return result


def packet(sdk,ph,pk,motor,target):
    if motor not in ACTIVE:raise ValueError('Only right IDs2-4 may receive reach targets')
    comm,error=pk.writeTxRx(ph,motor,42,6,[target & 255,target >> 8,0,0,30,0])
    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Reach acknowledgment failed; no retry')
    if rt.jc.read(sdk,ph,pk,motor,'goal')!=target:raise RuntimeError('Reach target readback mismatch')


def execute(sdk,ph,pk,group,initial,p,observer=None):
    if p.get('execution_policy') not in (None,'left_shoulder_leading_v1','left_coarse_lift_v2'):raise ValueError('Unsupported reach execution policy; obtain a new preview')
    coarse=p.get('execution_policy')=='left_coarse_lift_v2'
    leading=coarse or p.get('execution_policy')=='left_shoulder_leading_v1'
    if leading and (observer is None or (not coarse and (p.get('intermediate_tolerance_ticks')!=8 or p.get('final_tolerance_ticks')!=4)) or (coarse and p.get('joint_tolerances')!={'2':8,'3':16,'4':4}) or len(p.get('stage_minimum_heights_m',[]))!=len(p['waypoints'])):raise ValueError('Leading plan requires model observer and fixed tracking policy')
    if coarse and (p.get('coarse') is not True or not 10<=p['step_mm']<=20 or p['stage_minimum_heights_m'][-1]!=p['step_mm']/2000):raise ValueError('Fixed coarse lift completion policy required')
    attempted=set();commands={str(m):s['goal'] for m,s in initial.items()};trace=[];recent={str(m):[] for m in ACTIVE};cleanup={'attempted':False}
    def monitor(stage):
        states=rt.ah.snapshot(sdk,ph,pk)
        for mid,s in states.items():
            baseline=initial[mid];lo,hi=group['joints'][mid]['safe_raw_limits']
            if s['model']!=777 or s['mode']!=0 or s['torque']!=1 or [s['min'],s['max']]!=[baseline['min'],baseline['max']] or not lo<=s['position']<=hi or not 110<=s['voltage_present_raw']<=130 or s['temperature_raw']>=60 or s['goal']!=commands[mid]:raise RuntimeError('Joint target/state/power changed during reach')
            if int(mid) in ACTIVE:
                a=p['expected_positions'][mid];b=p['targets'][mid]
                if not min(a,b)-8<=s['position']<=max(a,b)+8:raise RuntimeError('Joint escaped reach window')
                row={'elapsed_s':time.monotonic(),'stage':stage,'target_raw':commands[mid],'goal_raw':s['goal'],'position_raw':s['position'],'moving':s['moving'],'torque':s['torque'],'voltage_raw':s['voltage_present_raw'],'temperature_c':s['temperature_raw']}
                for name in ('present_speed_raw','present_load_raw','servo_status_raw','motor_phase_current_raw'):
                    addr,size=rt.jc.STATUS_REGS[name];row[name]=read_extra(sdk,ph,pk,int(mid),addr,size)
                if row['servo_status_raw']:raise RuntimeError('Servo status fault during reach')
                recent[mid].append(row);del recent[mid][:-3]
            elif abs(s['position']-baseline['position'])>8 or s['moving']:raise RuntimeError('An untouched joint moved')
        delta=observer(states) if observer is not None else None
        if len(trace)<80:trace.append({'stage':stage,'positions':{m:s['position'] for m,s in states.items()},'goals':commands.copy(),'encoder_model_displacement_m':delta})
        if delta is not None and (delta[2]<-.0005 or delta[2]>p['step_mm']/1000+.0015 or math.hypot(delta[0],delta[1])>.0015):raise RuntimeError('Encoder-model gripper progress moved outside the upward corridor')
        return states
    try:
        for stage,point in enumerate(p['waypoints'],1):
            monitor(stage)
            for motor in ACTIVE:
                mid=str(motor)
                if point[mid]!=commands[mid]:
                    attempted.add(motor);commands[mid]=point[mid]
                    packet(sdk,ph,pk,motor,point[mid])
            deadline=time.monotonic()+1.5;stable=0
            tolerance=8 if leading and stage<len(p['waypoints']) else 4
            tolerances=p['joint_tolerances'] if coarse else {str(m):tolerance for m in ACTIVE}
            continue_tracking=coarse and stage<len(p['waypoints'])
            required_samples=1 if continue_tracking else 3
            while time.monotonic()<deadline:
                states=monitor(stage)
                progress_ok=not leading or observer(states)[2]>=p['stage_minimum_heights_m'][stage-1]
                # Lag sampling covers positions between baseline and each
                # intermediate target. Do not advance from an unsampled overshoot.
                if leading and (coarse or stage<len(p['waypoints'])):
                    progress_ok=progress_ok and all(min(p['expected_positions'][str(m)],point[str(m)])<=states[str(m)]['position']<=max(p['expected_positions'][str(m)],point[str(m)]) for m in ACTIVE)
                stable=stable+1 if progress_ok and all(abs(states[str(m)]['position']-point[str(m)])<=tolerances[str(m)] and (continue_tracking or not states[str(m)]['moving']) for m in ACTIVE) else 0
                if stable>=required_samples:break
                time.sleep(.05)
            else:
                if leading:
                    residuals={str(m):point[str(m)]-states[str(m)]['position'] for m in ACTIVE}
                    height=observer(states)[2]
                    raise TimeoutError(f'Reach stage{stage} stopped: goal-minus-position ticks={residuals}, allowed={tolerances}; modeled height={height*1000:.3f}mm, required={p["stage_minimum_heights_m"][stage-1]*1000:.3f}mm. No next waypoint attempted')
                raise TimeoutError('Reach waypoint did not settle; no next waypoint attempted')
        delta=observer(states) if observer is not None else None
        if delta is not None and delta[2]<(p['stage_minimum_heights_m'][-1] if coarse else p['step_mm']/1000-.001):raise RuntimeError('Joint endpoints settled but model gripper height is short; not completed')
        return {'completed':True,'completion_kind':'coarse_model_lift' if coarse else 'strict_model_lift','trajectory_mode':'continuous_bounded_tracking' if coarse else 'settled_waypoints','requested_lift_mm':p['step_mm'],'minimum_accepted_model_lift_mm':p['stage_minimum_heights_m'][-1]*1000 if coarse else p['step_mm']-1,'encoder_model_displacement_m':delta,'plan':p,'states':states,'trace':trace,'torque':'left on','note':'Encoder completion is not a measured Cartesian displacement; operator confirms direction and clearance.'}
    except BaseException as exc:
        evidence={str(m):stationary_timeout_evidence(recent[str(m)],commands[str(m)],stage) for m in attempted}
        if isinstance(exc,TimeoutError) and attempted and all(evidence.values()):
            cleanup.update({'action':'preserve_existing_targets','target_write_sent':False,'verified_stationary':evidence})
        elif attempted:
            cleanup['attempted']=True;cleanup['holds']={}
            old={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                for motor in sorted(attempted):
                    try:
                        if rt.jc.read(sdk,ph,pk,motor,'torque')!=1:raise RuntimeError('Torque not on; not re-enabling')
                        pos=rt.jc.read(sdk,ph,pk,motor,'position');mid=str(motor);lo,hi=group['joints'][mid]['safe_raw_limits'];a=p['expected_positions'][mid];b=p['targets'][mid]
                        if not lo<=pos<=hi or not min(a,b)-8<=pos<=max(a,b)+8:raise RuntimeError('Hold feedback outside reach window')
                        packet(sdk,ph,pk,motor,pos);cleanup['holds'][mid]={'acknowledged':True,'target':pos}
                    except Exception as error:cleanup['holds'][str(motor)]={'acknowledged':False,'error':str(error)}
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
        return {'completed':False,'error':str(exc) or type(exc).__name__,'plan':p,'trace':trace,'cleanup':cleanup,'note':'No automatic return or torque release; keep arm supported. Never retry uncertain reach automatically.'}


def make_observer(expected,cal):
    import numpy as np
    from lerobot.model.kinematics import RobotKinematics
    names=NAMES[:5]
    with solver_stdout_to_stderr():k=RobotKinematics(str(URDF),joint_names=list(names))
    def pose(raw):
        q=np.array([(raw[str(cal[n]['id'])]-(cal[n]['range_min']+cal[n]['range_max'])/2)*360/4095 for n in names])
        return k.forward_kinematics(q).copy()
    origin=pose(expected)
    def observe(states):
        now=pose({m:s['position'] for m,s in states.items()})
        delta=(now[:3,3]-origin[:3,3]).tolist()
        if not all(math.isfinite(v) for v in delta):raise RuntimeError('Non-finite model progress')
        return delta
    return observe


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['preview','execute']);parser.add_argument('--expected-positions',required=True);parser.add_argument('--plan-hash');args=parser.parse_args()
    group=json.loads(rt.jc.CONFIG.read_text())['groups']['right-arm'];expected=json.loads(args.expected_positions)
    cal_bytes=CAL.read_bytes();urdf_bytes=URDF.read_bytes();cal=json.loads(cal_bytes)
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            states=rt.ah.snapshot(sdk,ph,pk);offsets=calibration_offsets(sdk,ph,pk)
            p=plan(group,states,expected,cal,offsets,cal_bytes,urdf_bytes,args.action=='execute')
            if args.action=='preview':result={'completed':True,'plan':p,'hardware_writes':False}
            else:
                if args.plan_hash!=p['plan_hash']:raise ValueError('Preview identity changed; obtain a new preview')
                # IK may take time: re-read before any target packet.
                fresh=rt.ah.snapshot(sdk,ph,pk);preflight(group,fresh,expected,cal,calibration_offsets(sdk,ph,pk),True)
                result=execute(sdk,ph,pk,group,fresh,p,make_observer(expected,cal))
            print(json.dumps(result),flush=True);return 0 if result['completed'] else 1
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
