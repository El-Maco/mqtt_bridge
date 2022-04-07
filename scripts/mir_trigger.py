#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Int32, Float64
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

goal_x, goal_y = 11.8154, 9.47028
# orient_z, orient_w = 0.9985, -0.0552

class Mir_Trigger():

    # TODO: Cartesian to quaternion calc
    def __init__(self):

        self.target_point = Point(goal_x, goal_y, 0.0)
        x, y, z, w = quaternion_from_euler(0, 0, 3.0311392) # Change to pi
        self.quaternion = Quaternion(x, y, z, -w) # Inverting quaterion for multiplication
        

        self.pickup_goal = MoveBaseActionGoal()
        # self.pickup_goal.goal.target_pose.pose.position.x = goal_x
        self.pickup_goal.goal.target_pose.pose.position = self.target_point
        self.pickup_goal.goal.target_pose.pose.orientation = self.quaternion

        self.pickup_goal.goal.target_pose.header.frame_id = 'map'

        rospy.loginfo("Target point: ".format(self.target_point))
        rospy.loginfo("Target quaternion: ".format(self.quaternion))

        rospy.init_node('mir_trigger')

        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        rospy.Subscriber('/pickup', Int32, self._callback)
        rospy.loginfo("Subscribing to /pickup")
        rospy.spin()

    def _callback(self, data):
        rospy.loginfo("{}: I heard {}".format(rospy.get_caller_id(), data.data))
        pickup = data.data
        if pickup:
            # self.pickup_goal.goal.target_pose.pose.position.x = float(pickup) + 0.8154
            self.pub.publish(self.pickup_goal)
            rospy.loginfo("Proceeding to move MiR to goal:\n {}".format(self.pickup_goal.goal.target_pose.pose.position))

if __name__ == '__main__':
    Mir_Trigger()
