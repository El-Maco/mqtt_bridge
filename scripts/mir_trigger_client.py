#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionGoal


goal_x, goal_y = 11.8154, 9.47028
orient_z, orient_w = 0.9985, -0.0552
pickup_goal = MoveBaseActionGoal()
pickup_goal.goal.target_pose.pose.position.x = goal_x
pickup_goal.goal.target_pose.pose.position.y = goal_y
pickup_goal.goal.target_pose.pose.orientation.z = orient_z
pickup_goal.goal.target_pose.pose.orientation.w = orient_w
pickup_goal.goal.target_pose.header.frame_id = "map"

def callback(data):
    rospy.loginfo("{}: I heard {}".format(rospy.get_caller_id(), data.data))
    pickup = data.data
    if pickup:
        pub.publish(pickup_goal)

        rospy.wait_for_service('/mir_trigger_service')

        mir_trigger_service = rospy.ServiceProxy('/mir_trigger_service', Trigger)

        trig_req = TriggerRequest()

        res = mir_trigger_service(trig_req)

        print(res)

rospy.init_node('mir_trigger_client')
pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
rospy.Subscriber('/pickup', Bool, callback)
rospy.spin()


