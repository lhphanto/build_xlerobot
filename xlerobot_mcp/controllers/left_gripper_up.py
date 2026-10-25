#!/usr/bin/env python3
"""Parameterized2-10mm left gripper +model-Z IK step, using existing LeRobot solver."""
import argparse,fcntl,hashlib,itertools,json,math,os,signal,sys,tempfile,time
import right_gripper_up as reach


def _checked_plan(expected,cal,goals,distance_mm,coarse=False,endpoint=None):
    """Finite measured-pose plan; no hardware and no target-error compensation."""
    import numpy as np
    import xml.etree.ElementTree as ET
    from lerobot.model.kinematics import RobotKinematics
    tolerances={'2':8,'3':16,'4':4} if coarse else {'2':8,'3':8,'4':8}
    result=endpoint if endpoint is not None else reach.solve(expected,cal,step_mm=distance_mm,orientation_weight=0,lag_guard=False,**({'coarse':True} if coarse else {}))
    names=reach.NAMES[:5]
    with reach.solver_stdout_to_stderr():model=RobotKinematics(str(reach.URDF),joint_names=list(names))
    limits={j.attrib['name']:[float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper'])] for j in ET.parse(reach.URDF).getroot().findall('joint') if j.find('limit') is not None}
    def pose(raw):
        q=np.array([(raw[str(cal[n]['id'])]-(cal[n]['range_min']+cal[n]['range_max'])/2)*360/4095 for n in names])
        if any(not limits[n][0]<=np.deg2rad(q[i])<=limits[n][1] for i,n in enumerate(names)):raise ValueError('Leading path outside URDF limits')
        return model.forward_kinematics(q).copy()
    origin=pose(expected);target=result['targets']
    if coarse:
        # Pick a leader from FK at this pose. Shoulder-first can lower the
        # gripper in a folded pose even though the combined IK target is up.
        leaders=[]
        for mid in map(str,reach.ACTIVE):
            travel=target[mid]-expected[mid]
            if abs(travel)<16:continue
            direction=1 if travel>0 else -1
            first={m:goals[m] for m in target};first[mid]=expected[mid]+direction*16
            raw=expected|first;t=pose(raw);d=t[:3,3]-origin[:3,3]
            angle=float(np.rad2deg(np.arccos(np.clip((np.trace(origin[:3,:3].T@t[:3,:3])-1)/2,-1,1))))
            if d[2]<=.0002 or np.linalg.norm(d[:2])>.001 or angle>6:continue
            arrivals=[]
            for motor in target:
                change=first[motor]-expected[motor];sign=1 if change>=0 else -1
                arrivals.append([first[motor]-sign*lag for lag in range(min(tolerances[motor],abs(change))+1)])
            minimum=min(float((pose(expected|dict(zip(target,values)))[:3,3]-origin[:3,3])[2]) for values in itertools.product(*arrivals))
            if minimum>=-.0005:leaders.append((minimum,float(d[2]),mid,first))
        if not leaders:raise ValueError('No upward-leading joint within existing tracking and path limits')
        _,_,leader,first=max(leaders,key=lambda row:(row[0],row[1],row[2]))
        waypoints=[first]
        # Establish a bounded upward buffer before coordinating the other axes.
        travel=target[leader]-expected[leader];direction=1 if travel>0 else -1
        lead_end=expected[leader]+direction*min(32,abs(travel))
        if first[leader]!=lead_end:
            buffered=first.copy();buffered[leader]=lead_end;waypoints.append(buffered)
        prior=waypoints[-1]
    else:
        leader='2';first={m:goals[m] for m in target}
        travel=target['2']-expected['2']
        first['2']=expected['2']+max(-16,min(16,travel))
        if first['2']==goals['2'] or (travel and (first['2']-goals['2'])*travel<=0):
            raise ValueError('Requested lift cannot advance shoulder beyond its existing target; choose a larger bounded distance')
        waypoints=[first];prior=first
    steps=max(1,math.ceil(max(abs(target[m]-prior[m]) for m in target)/16))
    if prior!=target:
        for step in range(1,steps+1):
            waypoints.append({m:int(round(prior[m]+(target[m]-prior[m])*step/steps)) for m in target})
    if len(waypoints)>(6 if coarse else 4):raise ValueError('Leading path exceeds four stages')
    previous=expected.copy();last_height=0.;worst=0.;max_side=0.;heights=[];arrival_minima=[];arrival_maxima=[]
    for point in waypoints:
        raw=expected|point;t=pose(raw);d=t[:3,3]-origin[:3,3]
        angle=float(np.rad2deg(np.arccos(np.clip((np.trace(origin[:3,:3].T@t[:3,:3])-1)/2,-1,1))))
        if d[2]<last_height-.0002 or not -.0002<=d[2]<=distance_mm/1000+.001 or np.linalg.norm(d[:2])>.001 or angle>(6 if coarse else 3):raise ValueError('Shoulder-leading nominal path is not a small upward reach')
        if any(abs(point[m]-previous[m])>16 or abs(point[m]-expected[m])>(64 if coarse else 32) for m in target):raise ValueError('Leading stage exceeds joint bounds')
        # Include every integer lag of0..8ticks toward baseline, not just one
        # assumed arrival position. Sample response fractions independently.
        starts=[]
        for m in target:
            travel=previous[m]-expected[m];direction=1 if travel>=0 else -1
            starts.append(sorted({previous[m]-direction*lag for lag in range(min(tolerances[m],abs(travel))+1)}))
        partial_poses=set()
        for start_values in itertools.product(*starts):
            for fractions in itertools.product((0,.5,1),repeat=3):
                partial=expected.copy()
                for m,value,fraction in zip(target,start_values,fractions):partial[m]=int(round(value+fraction*(point[m]-value)))
                partial_poses.add(tuple(partial[str(m)] for m in reach.ACTIVE))
        for values in partial_poses:
            partial=expected|dict(zip(map(str,reach.ACTIVE),values))
            partial_pose=pose(partial);delta=partial_pose[:3,3]-origin[:3,3]
            partial_angle=float(np.rad2deg(np.arccos(np.clip((np.trace(origin[:3,:3].T@partial_pose[:3,:3])-1)/2,-1,1))))
            worst=min(worst,float(delta[2]));max_side=max(max_side,float(np.linalg.norm(delta[:2])))
            if delta[2]<-.0005 or delta[2]>distance_mm/1000+.0015 or np.linalg.norm(delta[:2])>.0015 or partial_angle>(6 if coarse else 3):raise ValueError('Leading path unsafe under bounded independent joint lag')
        # Derive arrival progress from the SAME integer0..8tick lag
        # envelope accepted by execute(). Nominal height minus a fixed3mm
        # wrongly assumes every joint has moved; an elbow still within8ticks
        # can account for more than3mm of that nominal height.
        arrivals=[]
        for m in target:
            travel=point[m]-expected[m];direction=1 if travel>=0 else -1
            arrivals.append([point[m]-direction*lag for lag in range(min(tolerances[m],abs(travel))+1)])
        arrival_heights=[float((pose(expected|dict(zip(target,values)))[:3,3]-origin[:3,3])[2]) for values in itertools.product(*arrivals)]
        minimum=min(arrival_heights);maximum=max(arrival_heights)
        if coarse and maximum>distance_mm/1000+.0015:raise ValueError('Arrival lag can exceed existing upward corridor')
        arrival_minima.append(minimum);arrival_maxima.append(maximum)
        heights.append(float(d[2]));last_height=float(d[2]);previous=raw
    # No stage can advance with zero observed lift. Final height/accuracy
    # requirements remain unchanged; intermediate floors match allowed lag.
    final_floor=distance_mm/2000 if coarse else distance_mm/1000-.001
    if coarse and arrival_minima[-1]<final_floor:raise ValueError('Tracking uncertainty leaves less than half the requested lift; choose a larger bounded distance or improve tracking')
    floors=[max(.0002,h) for h in arrival_minima[:-1]]+[final_floor]
    return {**result,'waypoints':waypoints,'lag_guard':True,'worst_modeled_partial_height_m':worst,
            'max_modeled_partial_side_m':max_side,'execution_policy':'left_coarse_lift_v2' if coarse else 'left_shoulder_leading_v1',
            **({'lead_joint_id':int(leader),'trajectory_mode':'continuous_bounded_tracking','joint_tolerances':tolerances,'predicted_final_model_lift_interval_mm':[arrival_minima[-1]*1000,arrival_maxima[-1]*1000],'coarse':True} if coarse else {}),
            'stage_minimum_heights_m':floors,'stage_nominal_heights_m':heights,'stage_arrival_minimum_heights_m':arrival_minima,'stage_arrival_maximum_heights_m':arrival_maxima,'intermediate_tolerance_ticks':None if coarse else 8,'final_tolerance_ticks':None if coarse else 4}


def shoulder_leading_plan(expected,cal,goals,distance_mm,coarse=False):
    if not coarse:
        return _checked_plan(expected,cal,goals,distance_mm)
    from coarse_lift_targets import candidates, MAX_PATH_CHECKS, PLANNING_MARGIN_M
    required=distance_mm/2000+PLANNING_MARGIN_M
    alternatives=candidates(expected,cal,distance_mm)
    attempts=0;rejections=[]
    for endpoint in alternatives:
        if endpoint['endpoint_minimum_m']<required:
            continue
        attempts+=1
        try:
            result=_checked_plan(expected,cal,goals,distance_mm,True,endpoint)
            if result['stage_arrival_minimum_heights_m'][-1]<required:
                raise ValueError('Checked arrival margin below required planning margin')
            result.update(target_selection='lerobot_fk_tracking_margin_v1',
                          planning_margin_mm=PLANNING_MARGIN_M*1000,
                          achieved_planning_margin_mm=(result['stage_arrival_minimum_heights_m'][-1]-distance_mm/2000)*1000,
                          candidate_path_checks=attempts)
            return result
        except ValueError as exc:
            rejections.append(str(exc))
        if attempts>=MAX_PATH_CHECKS:
            break
    detail='; '.join(rejections[:MAX_PATH_CHECKS]) or 'endpoint uncertainty too large'
    raise ValueError(f'No bounded lift with1mm planning margin: {detail}')


def make_plan(group,states,expected,offsets,execute=False,distance_mm=5,coarse=False):
    if type(distance_mm) not in (int,float) or not math.isfinite(distance_mm) or not (10<=distance_mm<=20 if coarse else 2<=distance_mm<=10):raise ValueError('distance_mm must be finite:2-10 strict or10-20 coarse')
    if type(coarse) is not bool:raise ValueError('coarse must be boolean')
    cal=group.get('kinematic_calibration')
    if not isinstance(cal,dict):raise ValueError('Capture/verify left calibration before Cartesian planning')
    cal_bytes=json.dumps(cal,sort_keys=True).encode();goals={m:s['goal'] for m,s in states.items()}
    result=reach.plan(group,states,expected,cal,offsets,cal_bytes,reach.URDF.read_bytes(),execute,
                      solver=lambda e,c:shoulder_leading_plan(e,c,goals,distance_mm,**({'coarse':True} if coarse else {})),step_mm=distance_mm,goal_error_limit=8,waypoint_limit=16,coarse=coarse)
    # Stored goals influence stage1 even when measured positions are unchanged.
    result['plan_hash']=hashlib.sha256(json.dumps({'base_plan_hash':result['plan_hash'],'starting_goals':goals},sort_keys=True).encode()).hexdigest()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['preview','execute']);parser.add_argument('--expected-positions',required=True);parser.add_argument('--plan-hash');parser.add_argument('--distance-mm',type=float,default=5);parser.add_argument('--coarse',action='store_true');args=parser.parse_args()
    group=json.loads(reach.rt.jc.CONFIG.read_text())['groups']['left-arm'];expected=json.loads(args.expected_positions)
    import scservo_sdk as sdk
    port=group['port'];name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):raise RuntimeError('Port setup failed')
            def timeout(length):ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout;time.sleep(.1)
            states=reach.rt.ah.snapshot(sdk,ph,pk);offsets=reach.calibration_offsets(sdk,ph,pk)
            p=make_plan(group,states,expected,offsets,args.action=='execute',distance_mm=args.distance_mm,coarse=args.coarse)
            if args.action=='preview':result={'completed':True,'plan':p,'hardware_writes':False}
            else:
                if args.plan_hash!=p['plan_hash']:raise ValueError('Preview identity changed; obtain a new preview')
                observer=reach.make_observer(expected,group['kinematic_calibration'])
                fresh=reach.rt.ah.snapshot(sdk,ph,pk)
                reach.preflight(group,fresh,expected,group['kinematic_calibration'],reach.calibration_offsets(sdk,ph,pk),True,goal_error_limit=8,coarse=args.coarse)
                if any(fresh[m]['goal']!=states[m]['goal'] for m in states):raise ValueError('Stored goals changed after planning; obtain a new preview')
                result=reach.execute(sdk,ph,pk,group,fresh,p,observer)
            print(json.dumps(result),flush=True);return 0 if result['completed'] else 1
        finally:ph.closePort()
if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(main())
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
