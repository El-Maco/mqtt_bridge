from abc import ABCMeta
from typing import Optional, Type, Dict, Union

import inject
import paho.mqtt.client as mqtt
import rospy
from .util import lookup_object, extract_values, populate_instance, get_quaternion_from_euler

from std_msgs.msg import String, Bool, Int32, Float64
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion
from math import pi
from mqtt_bridge.msg import MirPosition

# TODO: ros_msg should be json/dict
def format_rosmsg(msg_type, msg_dict):
    # ROS message requires quaternion calculations from euler coordinates
    if (msg_type == type(MoveBaseActionGoal())):

        ros_msg = MoveBaseActionGoal()
        ros_msg.goal.target_pose.header.frame_id = 'map'

        target_point = Point(msg_dict['x'], msg_dict['y'], 0.0)

        x, y, z, w = get_quaternion_from_euler(0, 0, msg_dict['theta'])
        rospy.loginfo("Quaternion: [{}, {}, {}, {}]".format(x, y, z, w))
        quaternion = Quaternion(x, y, z, w)

        ros_msg.goal.target_pose.pose.position = target_point
        ros_msg.goal.target_pose.pose.orientation = quaternion
    else:
        ros_msg = populate_instance(msg_dict, msg_type())

    return ros_msg


def create_bridge(factory: Union[str, "Bridge"], msg_type: Union[str, Type[rospy.Message]], topic_from: str,
                  topic_to: str, frequency: Optional[float] = None, **kwargs) -> "Bridge":
    """ generate bridge instance using factory callable and arguments. if `factory` or `meg_type` is provided as string,
     this function will convert it to a corresponding object.
    """
    if isinstance(factory, str):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, str):
        msg_type = lookup_object(msg_type)
    if not issubclass(msg_type, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "reprensentation")
    return factory(
        topic_from=topic_from, topic_to=topic_to, msg_type=msg_type, frequency=frequency, **kwargs)


class Bridge(object, metaclass=ABCMeta):
    """ Bridge base class """
    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr('serializer')
    _deserialize = inject.attr('deserializer')
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    """ Bridge from ROS topic to MQTT

    bridge ROS messages on `topic_from` to MQTT topic `topic_to`. expect `msg_type` ROS message type.
    """

    def __init__(self, topic_from: str, topic_to: str, msg_type: rospy.Message, frequency: Optional[float] = None):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._last_published = rospy.get_time()
        self._interval = 0 if frequency is None else 1.0 / frequency
        rospy.Subscriber(topic_from, msg_type, self._callback_ros)

    def _callback_ros(self, msg: rospy.Message):
        rospy.logdebug("ROS received from {}".format(self._topic_from))
        now = rospy.get_time()
        if now - self._last_published >= self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg: rospy.Message):
        rospy.loginfo("extracted value before serialization: {}".format(extract_values(msg)))

        # For specific topic only publish result value (int)
        if (self._topic_to == "/move_result"):
            try:
                result = msg.status.status
                payload = self._serialize(result)
                rospy.loginfo(result)
            except Exception as e:
                rospy.logerr("Failed to get result status message from {}".format(type(msg)))
                rospy.logerr(e)

        else:
            payload = self.serialize(extract_values(msg))
            rospy.loginfo("Serialized payload:", payload)

        rospy.loginfo("final payload to publish: {}".format(payload))
        rospy.loginfo("type: {}".format(type(payload)))
        self._mqtt_client.publish(topic=self._topic_to, payload=payload)

class MqttToRosBridge(Bridge):
    """ Bridge from MQTT to ROS topic

    bridge MQTT messages on `topic_from` to ROS topic `topic_to`. MQTT messages will be converted to `msg_type`.
    """

    def __init__(self, topic_from: str, topic_to: str, msg_type: Type[rospy.Message],
                 frequency: Optional[float] = None, queue_size: int = 10):
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
        # Adding the correct topic to subscribe to
        self._mqtt_client.subscribe(self._topic_from)
        self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size)
        rospy.loginfo("Initialization of MqttToRosBridge SUCCESSFUL")

    def _callback_mqtt(self, client: mqtt.Client, userdata: Dict, mqtt_msg: mqtt.MQTTMessage):
        """ callback from MQTT """
        rospy.logdebug("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()

        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg: mqtt.MQTTMessage) -> rospy.Message:
        """ create ROS message from MQTT payload """
        # Hack to enable both, messagepack and json deserialization.
        if self._serialize.__name__ == "packb":
            msg_dict = self._deserialize(mqtt_msg.payload, raw=False)
        else:
            rospy.loginfo("------Deserializing-------")
            try:
                msg_dict = self._deserialize(mqtt_msg.payload.decode('utf-8').replace("'", '"'))
                ros_msg = format_rosmsg(self._msg_type, msg_dict)
                rospy.loginfo("Created ROS message")


            except Exception as e:
                rospy.logwarn(e)
                rospy.loginfo("Failed to deserialize payload:")
                raise

            # populates the dictionary into the specified ROS Message
            return ros_msg


__all__ = ['create_bridge', 'Bridge', 'RosToMqttBridge', 'MqttToRosBridge']
