#!/bin/sh
rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal '{header: {seq: 0, stamp: {secs: 369, nsecs: 9000000}, frame_id: ''}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}, goal: {target_pose: {header: {seq: 0, stamp: {secs: 369, nsecs: 993000000}, frame_id: 'map'}, pose: {position: {x: 11.3160, y: 11.4400, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.6677, w: 0.7443}}}}}'
