#!/usr/bin/env python3
"""Preview or execute one supervised right-arm move to its fixed start pose; IDs2-4 only, then hold."""
import json,signal,sys
import left_start_pose as sp

# Hand-set pose the elbow trials ran from; IDs5 and6 receive no writes.
START={'2':1105,'3':2990,'4':2230}
RIGHT={'group':'right-arm','policy':'right_start_pose_v1','start':START,
       # At P32 the elbow stalls about16ticks short while lifting, and the one trim closes that.
       'stage_tolerance':{'2':16,'3':24,'4':16},
       'max_goal_offset':{'2':8,'3':16,'4':8},
       'calibration':lambda group:json.loads(sp.reach.CAL.read_text())}

if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:sys.exit(sp.main(RIGHT))
    except (Exception,KeyboardInterrupt) as exc:print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
