#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseActionGoal


goal_x, goal_y = 11.8154, 9.47028
orient_z, orient_w = 0.9985, -0.0552

class Mir_Trigger():

    def __init__(self):
        self.pickup_goal = MoveBaseActionGoal()
        self.pickup_goal.goal.target_pose.pose.position.x = goal_x
        self.pickup_goal.goal.target_pose.pose.position.y = goal_y
        self.pickup_goal.goal.target_pose.pose.orientation.z = orient_z
        self.pickup_goal.goal.target_pose.pose.orientation.w = orient_w

        rospy.init_node('mir_trigger')

        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        rospy.Subscriber('/pickup', Bool, self._callback)
        rospy.loginfo("Subscribing to /pickup")
        rospy.spin()

    def _callback(self, data):
        rospy.loginfo("{}: I heard {}".format(rospy.get_caller_id(), data.data))
        pickup = data.data
        if pickup:
            self.pub.publish(self.pickup_goal)
            rospy.loginfo("Proceeding to move MiR to goal:\n {}".format(self.pickup_goal.goal.target_pose.pose.position))

if __name__ == '__main__':
    Mir_Trigger()
