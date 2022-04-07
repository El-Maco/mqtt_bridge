#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool

def trigger_response(req):
    data = Bool()
    data.data = False
    rospy.loginfo("Publishing to /pickup: {}".format(data))
    pub = rospy.Publisher('/pickup', Bool, queue_size=1)
    rospy.sleep(0.25)
    pub.publish(data)
    rospy.loginfo("Publish successful")
    return TriggerResponse(
            success=True,
            message="MIR100 successfully triggered"
            )
def mir_trigger_server():
    rospy.init_node('mir_trigger_service')
    my_service = rospy.Service('/mir_trigger_service', Trigger, trigger_response)
    rospy.spin()

if __name__ == '__main__':
    mir_trigger_server()
