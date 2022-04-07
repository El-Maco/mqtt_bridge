#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool, Int32, Float64
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler


# goal_x, goal_y = 11.8154, 9.47028
# orient_z, orient_w = 0.9985, -0.0552

target_point = Point(11.8154, 9.47028, 0.0)
q = quaternion_from_euler(0, 0, -137.678)
quaternion = Quaternion(q[0], q[1], q[2], q[3])
        
pickup_goal = MoveBaseActionGoal()
pickup_goal.goal.target_pose.pose.position = target_point
pickup_goal.goal.target_pose.pose.orientation = quaternion

pickup_goal.goal.target_pose.header.frame_id = "map"

def callback(data):
    rospy.loginfo("{}: I heard {}".format(rospy.get_caller_id(), data.data))
    pickup = data.data
    rospy.loginfo("msg_type: {}".format(type(pickup)))
    if pickup:
        pub.publish(pickup_goal)

        rospy.wait_for_service('/mir_trigger_service')

        mir_trigger_service = rospy.ServiceProxy('/mir_trigger_service', Trigger)

        trig_req = TriggerRequest()

        res = mir_trigger_service(trig_req)

        print(res)

rospy.init_node('mir_trigger_client')
pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
rospy.Subscriber('/pickup', Int32, callback)
rospy.spin()


