#!/usr/bin/env python3
"""Preview or execute one supervised left-arm reach of the gripper frame to a point in the arm base frame; IDs1-4, then hold."""
import argparse,fcntl,hashlib,json,math,os,signal,sys,tempfile,time
import numpy as np
import joint_control as jc
import left_arm_hold as ah
import left_start_pose as sp
import right_gripper_up as reach

POLICY='left_reach_v2'
ACTIVE=('1','2','3','4')  # Yaw, shoulder, elbow, wrist flex; wrist roll and gripper keep their pose.
# Arm base frame (SO101 model, meters): x forward, y left, z up. Targets outside are refused.
WORKSPACE={'x':(.08,.45),'y':(-.30,.30),'z':(-.10,.45)}
MAX_TRAVEL=400        # ticks per joint (~35 degrees)
LEVEL_TOLERANCE_DEG=2.0  # jaw line within this of horizontal when level_jaws is requested
IK_TOLERANCE_M=.005
PATH_DIP_M=.02        # gripper may not sag below the lower end of the move by more than this
PATH_MARGIN_M=.03     # live envelope beyond the straight start-to-target box
# Top-down mode (grasps from above): gripper frame z (approach, out past the fingertips) points straight down, or leans
# approach_tilt_deg away from the arm base along the target's bearing; frame x is the jaw-closing line, set by wrist roll.
POINTING_TOLERANCE_DEG=3.0   # approach and jaw line within this of what was asked
MAX_APPROACH_TILT_DEG=90.0  # 90 = pointing horizontally outward; lets a reorientation start from a forward-pointing gripper
PATH_POINTING_MARGIN_DEG=10.0  # stages between two pointing poses may wander this much beyond the worse end
# Extended shoulder holding error reached24ticks on November17. Preserve that
# support with at most32ticks of goal offset; stage tolerance remains16.
MAX_GOAL_OFFSET={'1':8,'2':32,'3':16,'4':8,'5':8}
# A completed move may leave goal = target +/- MAX_GOAL_OFFSET with position within FINAL_TOLERANCE of target, so the
# next move's preflight must accept that much holding offset (a stationary joint; yaw finished 10 off on November16).
GOAL_TOLERANCE={m:v+sp.FINAL_TOLERANCE for m,v in MAX_GOAL_OFFSET.items()}
# A gripper closed on an object holds its goal up to the squeeze (max 24 ticks) past its position (November23: 9 after
# a 24-tick squeeze); a lift or carry must accept that. The reach never writes ID6.
GOAL_TOLERANCE['6']=24
ARM={'group':'left-arm','policy':POLICY,'stage_tolerance':{'1':8,'2':16,'3':16,'4':8,'5':8},
     'max_goal_offset':MAX_GOAL_OFFSET,'goal_tolerance':GOAL_TOLERANCE,
     'preserve_hold_offsets':True,
     # Lifting the extended arm: the shoulder stalled 14-20 ticks behind 16-tick stages (November16, P32, load 5-12%);
     # the yaw then stalled 11 behind against gearbox friction with the arm extended, and 9 behind even with a
     # 16-tick lead and nothing touching (request 9a63b30c), so it gets the shoulder's 32.
     'lead':{'1':32,'2':32,'3':16,'4':16,'5':16},'max_travel':MAX_TRAVEL,
     'calibration':lambda group:group['kinematic_calibration']}


def packet(sdk,ph,pk,motor,target):
    if motor not in (1,2,3,4,5):raise ValueError('Only left IDs1-5 may receive reach targets')
    comm,error=pk.writeTxRx(ph,motor,42,6,[target & 255,target >> 8,0,0,30,0])
    if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError('Reach acknowledgment failed; no retry')
    if jc.read(sdk,ph,pk,motor,'goal')!=target:raise RuntimeError('Reach target readback mismatch')


def kinematics(cal):
    from lerobot.model.kinematics import RobotKinematics
    names=reach.NAMES[:5]
    with reach.solver_stdout_to_stderr():model=RobotKinematics(str(reach.URDF),joint_names=list(names))
    mid={n:(cal[n]['range_min']+cal[n]['range_max'])/2 for n in names}
    to_deg=lambda raw:np.array([(raw[str(cal[n]['id'])]-mid[n])*360/4095 for n in names])
    to_raw=lambda deg:{str(cal[n]['id']):int(round(deg[i]*4095/360+mid[n])) for i,n in enumerate(names)}
    return model,to_deg,to_raw


def solve(cal,current,target):
    """Joint targets for IDs1-4 that put the gripper frame at target, keeping roll and gripper as they are."""
    model,to_deg,to_raw=kinematics(cal)
    q0=to_deg(current)
    start=model.forward_kinematics(q0).copy()
    wanted=start.copy();wanted[:3,3]=target
    # Position only: a 5-joint arm cannot also hold the gripper's attitude while it moves,
    # and even a small orientation weight left targets 5-17 mm short.
    q=model.inverse_kinematics(q0,wanted,position_weight=1.0,orientation_weight=0.0,max_iters=64)
    raw=to_raw(q)
    targets={m:raw[m] for m in ACTIVE}
    # Check the pose actually commanded: IDs1-4 from IK, roll held at its current value.
    commanded=dict(current);commanded.update(targets)
    reached=model.forward_kinematics(to_deg(commanded))[:3,3]
    error=float(np.linalg.norm(reached-target))
    if error>IK_TOLERANCE_M:raise ValueError(f'Target not reachable with the current wrist roll: closest model pose is {error*1000:.0f} mm away')
    return targets,start[:3,3],reached,error


def jaw_tilt_deg(model,to_deg,raw):
    """Height slope of the line between the jaws (degrees); negative means the movable jaw is lower."""
    model.forward_kinematics(to_deg(raw));r=model.robot
    axis=r.get_T_world_frame('moving_jaw_so101_v1_link')[:3,2];approach=r.get_T_world_frame('gripper_frame_link')[:3,2]
    opening=np.cross(axis,approach)
    return float(np.degrees(np.arcsin(opening[2]/np.linalg.norm(opening))))


def level_roll(cal,raw,limits):
    """Roll (raw) nearest the current one that makes the jaw line horizontal, within limits."""
    model,to_deg,_=kinematics(cal);best=None
    for roll in range(max(limits[0],raw['5']-MAX_TRAVEL),min(limits[1],raw['5']+MAX_TRAVEL)+1,4):
        tilt=abs(jaw_tilt_deg(model,to_deg,dict(raw,**{'5':roll})))
        if best is None or tilt<best[0]-1e-9:best=(tilt,roll)
    if best is None or best[0]>LEVEL_TOLERANCE_DEG:raise ValueError(f'Jaws cannot be levelled within {MAX_TRAVEL} roll ticks and saved limits (best {best and best[0]:.1f} deg)')
    return best[1],best[0]


def approach_direction(target,tilt_deg):
    """Unit approach vector: straight down, leaned tilt_deg away from the base toward the target's bearing."""
    radial=np.array([target[0],target[1],0.]);n=np.linalg.norm(radial)
    radial=radial/n if n>1e-6 else np.array([1.,0.,0.])
    t=math.radians(tilt_deg);return math.cos(t)*np.array([0.,0.,-1.])+math.sin(t)*radial


def jaw_line(approach,yaw_deg):
    """Unit jaw-closing direction at yaw_deg (arm frame, about z), made perpendicular to the approach."""
    d=np.array([math.cos(math.radians(yaw_deg)),math.sin(math.radians(yaw_deg)),0.]);d=d-(d@approach)*approach
    return d/np.linalg.norm(d)


def pointing_errors(model,to_deg,raw,approach,want_line=None):
    """(approach error deg, jaw-line error deg or None, gripper position) of a raw pose. Either jaw order counts."""
    T=model.forward_kinematics(to_deg(raw))
    tilt=math.degrees(math.acos(float(np.clip(T[:3,2]@approach,-1,1))))
    line=None if want_line is None else math.degrees(math.asin(float(np.clip(abs(np.cross(T[:3,0],want_line)@approach),0,1))))
    return tilt,line,T[:3,3].copy()


def solve_pointing(cal,current,target,limits,tilt_deg=0.,jaw_yaw_deg=None):
    """IDs1-5 targets putting the gripper frame at target with the approach pointing as asked (and the jaw line at
    jaw_yaw_deg if given), inside safe limits; the valid solution with the least joint travel. Raises if none."""
    from scipy.optimize import least_squares
    model,to_deg,to_raw=kinematics(cal)
    ids=('1','2','3','4','5');a=approach_direction(target,tilt_deg)
    want=None if jaw_yaw_deg is None else jaw_line(a,jaw_yaw_deg)
    q0=to_deg(current);lo=to_deg(dict(current,**{m:limits[m][0] for m in ids}));hi=to_deg(dict(current,**{m:limits[m][1] for m in ids}))
    def residual(q):
        T=model.forward_kinematics(q)
        r=[*((T[:3,3]-target)*1000),*((T[:3,2]-a)*100)]  # mm; ~1.7 per degree
        r.append(100*float(np.cross(T[:3,0],want)@a) if want is not None else .02*(q[4]-q0[4]))  # roll free: stay put
        return r
    best=None
    for roll_turn in (0.,180.,-180.):
        seed=q0.copy();seed[4]+=roll_turn
        if not np.all((lo<=seed)&(seed<=hi)):continue
        sol=least_squares(residual,np.clip(seed,lo+1e-6,hi-1e-6),bounds=(lo,hi),xtol=1e-10,ftol=1e-10,max_nfev=400)
        raw=to_raw(sol.x);targets={m:min(max(raw[m],limits[m][0]),limits[m][1]) for m in ids}
        tilt,line,reached=pointing_errors(model,to_deg,dict(current,**targets),a,want)
        error=float(np.linalg.norm(reached-target))
        if error>IK_TOLERANCE_M or tilt>POINTING_TOLERANCE_DEG or (line is not None and line>POINTING_TOLERANCE_DEG):continue
        travel=max(abs(targets[m]-current[m]) for m in ids)
        if best is None or travel<best[0]:best=(travel,targets,reached,error,tilt,line)
    if best is None:raise ValueError(f'Target not reachable pointing {tilt_deg:g} deg from straight down'+('' if jaw_yaw_deg is None else f' with the jaw line at {jaw_yaw_deg:g} deg')+f' within {IK_TOLERANCE_M*1000:.0f} mm and {POINTING_TOLERANCE_DEG:g} deg')
    _,targets,reached,error,tilt,line=best
    start=model.forward_kinematics(to_deg(current))[:3,3].copy()
    return targets,start,reached,error,{'approach_tilt_deg':float(tilt_deg),'approach_error_deg':tilt,'jaw_yaw_deg':jaw_yaw_deg,'jaw_line_error_deg':line,'approach':a.tolist()}


def make_plan(group,states,expected,target,observer,level=False,top_down=False,jaw_yaw_deg=None,approach_tilt_deg=0.):
    if level and top_down:raise ValueError('Choose level_jaws or top_down, not both')
    if not top_down and (jaw_yaw_deg is not None or approach_tilt_deg):raise ValueError('jaw_yaw_deg and approach_tilt_deg need top_down')
    if top_down:
        if type(approach_tilt_deg) not in (int,float) or not 0<=approach_tilt_deg<=MAX_APPROACH_TILT_DEG:raise ValueError(f'approach_tilt_deg must be 0-{MAX_APPROACH_TILT_DEG:g}')
        if jaw_yaw_deg is not None:
            if type(jaw_yaw_deg) not in (int,float) or not math.isfinite(jaw_yaw_deg):raise ValueError('jaw_yaw_deg must be a finite number')
            jaw_yaw_deg=round(float(jaw_yaw_deg)%180.,3)  # a jaw line: either jaw order is the same request
        approach_tilt_deg=float(approach_tilt_deg)
    target=np.asarray(target,dtype=float)
    if target.shape!=(3,) or not np.all(np.isfinite(target)):raise ValueError('Target must be three finite numbers (meters)')
    for axis,value in zip('xyz',target):
        lo,hi=WORKSPACE[axis]
        if not lo<=value<=hi:raise ValueError(f'Target {axis}={value:.3f} m outside the reach workspace {lo}..{hi} m')
    cal=group.get('kinematic_calibration')
    if not isinstance(cal,dict):raise ValueError('Left kinematic calibration required')
    current={m:s['position'] for m,s in states.items()}
    tilt=None;pointing=None
    if top_down:
        limits={m:group['joints'][m]['safe_raw_limits'] for m in ('1','2','3','4','5')}
        targets,start,reached,error,pointing=solve_pointing(cal,current,target,limits,approach_tilt_deg,jaw_yaw_deg)
        far={m:abs(targets[m]-current[m]) for m in targets if abs(targets[m]-current[m])>MAX_TRAVEL}
        if far:raise ValueError(f'Pointing pose needs {", ".join(f"ID{m} {t} ticks" for m,t in far.items())} of travel (max {MAX_TRAVEL}); '
                                'step there through a larger approach_tilt_deg first')
    elif level:
        # Level the jaws at the target: solve position, pick the roll, then re-solve IDs1-4 with that roll.
        first,*_=solve(cal,current,target)
        roll,_=level_roll(cal,dict(current,**first),group['joints']['5']['safe_raw_limits'])
        targets,start,reached,error=solve(cal,dict(current,**{'5':roll}),target)
        model,to_deg,_=kinematics(cal);start=model.forward_kinematics(to_deg(current))[:3,3]
        targets['5']=roll;tilt=jaw_tilt_deg(model,to_deg,dict(current,**targets))
        if abs(tilt)>LEVEL_TOLERANCE_DEG:raise ValueError(f'Levelled plan still tilts the jaws {tilt:.1f} deg')
    else:
        targets,start,reached,error=solve(cal,current,target)
    move=reached-start
    arm=dict(ARM,start=targets,packet=packet,
             max_height_m=abs(float(move[2]))+PATH_MARGIN_M,max_side_m=float(math.hypot(move[0],move[1]))+PATH_MARGIN_M)
    # Small near-vertical upward requests must actually rise; encoder errors
    # alone previously accepted nearly zero lift when the shoulder lagged.
    if .004<=float(move[2])<=.021 and math.hypot(move[0],move[1])<=.002:
        arm['minimum_final_height_m']=float(move[2])/2
    p=sp.make_plan(group,states,expected,observer,arm)
    floor=min(0.,float(move[2]))-PATH_DIP_M
    lowest=min([m['height_m'] for m in p['modeled_gripper']],default=0.)
    if lowest<floor:raise ValueError(f'Planned path sags {(floor+PATH_DIP_M-lowest)*1000:.0f} mm below its lower end; reach in two steps')
    if pointing:
        # Stages interpolate joints, so check the approach along them; a descent between two pointing poses must stay pointing.
        model,to_deg,_=kinematics(cal);a=np.array(pointing['approach'])
        start_error,_,_=pointing_errors(model,to_deg,current,a)
        path=[pointing_errors(model,to_deg,dict(current,**w),a)[0] for w in p['waypoints']]
        worst=max(path,default=pointing['approach_error_deg'])
        if start_error<=POINTING_TOLERANCE_DEG+2 and worst>max(start_error,pointing['approach_error_deg'])+PATH_POINTING_MARGIN_DEG:
            raise ValueError(f'Path between two pointing poses swings the approach {worst:.0f} deg off; reach in smaller steps')
        pointing.update(start_approach_error_deg=start_error,path_max_approach_error_deg=worst)
    p.update(target_m=target.tolist(),start_gripper_m=start.tolist(),modeled_reach_m=reached.tolist(),ik_error_mm=error*1000,
             move_m=move.tolist(),lowest_path_height_change_mm=lowest*1000,level_jaws=bool(level),jaw_tilt_deg=tilt,
             top_down=bool(top_down),pointing=pointing,
             note='Positions use the uncalibrated SO101 model and camera-to-arm calibration v1 (about 1 cm). Keep the path clear.')
    identity={'base':p['plan_hash'],'target':[round(v,4) for v in target],'level':bool(level)}
    if top_down:identity.update(top_down=True,jaw_yaw_deg=jaw_yaw_deg,approach_tilt_deg=approach_tilt_deg)
    p['plan_hash']=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    return p,arm


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['preview','execute'])
    parser.add_argument('--expected-positions',required=True);parser.add_argument('--target',type=float,nargs=3,required=True,metavar=('X','Y','Z'))
    parser.add_argument('--plan-hash');parser.add_argument('--level-jaws',action='store_true')
    parser.add_argument('--top-down',action='store_true');parser.add_argument('--jaw-yaw-deg',type=float);parser.add_argument('--approach-tilt-deg',type=float,default=0.)
    args=parser.parse_args()
    expected=json.loads(args.expected_positions)
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
            observer=reach.make_observer(expected,group['kinematic_calibration'])
            states=ah.snapshot(sdk,ph,pk);p,arm=make_plan(group,states,expected,args.target,observer,args.level_jaws,args.top_down,args.jaw_yaw_deg,args.approach_tilt_deg)
            if args.action=='preview':result={'completed':True,'plan':p,'hardware_writes':False}
            else:
                if args.plan_hash!=p['plan_hash']:raise ValueError('Preview identity changed; obtain a new preview')
                result=sp.execute(sdk,ph,pk,group,states,p,observer,arm)
            print(json.dumps(result),flush=True);return 0 if result['completed'] else 1
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
