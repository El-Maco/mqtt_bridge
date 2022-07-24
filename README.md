# mqtt_bridge

This project is forked from the [mqtt_bridge](//github.com/groove-x/mqtt_bridge) project, with modifications to suit the use case for triggering the MiR100 through MQTT.
The purpose of the modifications is to be able to integrate the ROS-operated MiR100 with IEC61499 control systems using the MQTT protocol as a middleware.

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.


## Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

This limitation can be overcome by defining custom bridge class, though.


## Demo

### Prerequisites

```
$ sudo apt install python3-pip
$ sudo apt install ros-noetic-rosbridge-library
$ sudo apt install mosquitto mosquitto-clients
```

### Install python modules

```bash
$ pip3 install -r requirements.txt
```

### launch node

``` bash
$ roslaunch mqtt_bridge demo.launch
```

Publish `{x: X, y: Y, theta: Theta}` to MQTT topic `/target_pos`,

```
$ ./commands/mqtt_pickup.sh
```

and see the command received in ROS topic `/move_base/goal`.

```
$ rostopic echo /move_base/goal
```

When the MiR100 completes the execution of the mission, a response can be seen in the ROS topic `/move_base/result`

```
$ rostopic sub /move_base/result
```

similarly, the result code is sent to the MQTT topic `/move_result`.
You can also see MQTT messages using `mosquitto_sub`

```
$ mosquitto_sub -v -t '#'
```

## Usage

parameter file (config.yaml):

``` yaml
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: 192.168.1.12 # broker address
    port: 1883
    keepalive: 60
bridge:
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: move_base_msgs.msg:MoveBaseActionGoal
    topic_from: /target_pos
    topic_to: /move_base/goal
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: move_base_msgs.msg:MoveBaseActionResult
    topic_from: /move_base/result
    topic_to: /move_result
```

It is possible to use any msg types like `sensor_msgs.msg:Imu`.

launch file:

``` xml
<launch>
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam file="/path/to/config.yaml" command="load" />
  </node>
</launch>
```


## Configuration

### mqtt

Parameters under `mqtt` section are used for creating paho's `mqtt.Client` and its configuration.

#### subsections

* `client`: used for `mqtt.Client` constructor
* `tls`: used for tls configuration
* `account`: used for username and password configuration
* `message`: used for MQTT message configuration
* `userdata`: used for MQTT userdata configuration
* `will`: used for MQTT's will configuration

See `mqtt_bridge.mqtt_client` for detail.

### mqtt private path

If `mqtt/private_path` parameter is set, leading `~/` in MQTT topic path will be replaced by this value. For example, if `mqtt/pivate_path` is set as "device/001", MQTT path "~/value" will be converted to "device/001/value".

### serializer and deserializer

`mqtt_bridge` uses `msgpack` as a serializer by default. But you can also configure other serializers. For example, if you want to use json for serialization, add following configuration.

``` yaml
serializer: json:dumps
deserializer: json:loads
```

### bridges

You can list ROS <--> MQTT tranfer specifications in following format.

``` yaml
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

* `factory`: bridge class for transfering message from ROS to MQTT, and vise versa.
* `msg_type`: ROS Message type transfering through the bridge.
* `topic_from`: topic incoming from (ROS or MQTT)
* `topic_to`: topic outgoing to (ROS or MQTT)

Also, you can create custom bridge class by inheriting `mqtt_brige.bridge.Bridge`.


## License

This software is released under the MIT License, see LICENSE.txt.
