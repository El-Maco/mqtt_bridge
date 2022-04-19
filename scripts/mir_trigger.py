#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Int32, Float64
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import pi
from mqtt_bridge.msg import MirPosition

goal_x, goal_y = 11.6605, 9.47028
# orient_z, orient_w = 0.9985, -0.0552

class Mir_Trigger():

    # TODO: Cartesian to quaternion calc
    def __init__(self):

        self.pickup_goal = MoveBaseActionGoal()

        self.pickup_goal.goal.target_pose.header.frame_id = 'map'

        rospy.init_node('mir_trigger')

        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        rospy.Subscriber('/pickup', MirPosition, self._callback)
        rospy.loginfo("Subscribing to /pickup")
        rospy.spin()


    def _callback(self, msg):
        rospy.loginfo("{}: I heard {}".format(rospy.get_caller_id(), msg))

        target_point = Point(msg.x, msg.y, 0.0)
        x, y, z, w = quaternion_from_euler(0, 0, msg.theta)
        rospy.loginfo("Quaternion: [{}, {}, {}, {}]".format(x, y, z, w))
        quaternion = Quaternion(x, y, z, w)

        self.pickup_goal.goal.target_pose.pose.position = target_point
        self.pickup_goal.goal.target_pose.pose.orientation = quaternion
        rospy.loginfo("Target point: ".format(target_point))
        rospy.loginfo("Target quaternion: ".format(quaternion))

        self.pub.publish(self.pickup_goal)
        rospy.loginfo("Proceeding to move MiR to goal:\n {}".format(self.pickup_goal.goal.target_pose.pose.position))

if __name__ == '__main__':
    Mir_Trigger()
