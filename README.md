# udp_example
Simple ROS2 node example to get data over UDP and publish it as a ROS topic

There is a little server which accepts any message and replies with a dict/JSON string mimicking
a ultrasonic range sensor

The ROS node periodically sends a message to the server and expects a JSON reply. That JSON reply
is massaged into a sensor_msg.mgs.Range message and published to a ROS topic. 

## Build

A simple *colcon build* should do it

## Run

*ros2 run udp_example udp_test_server* to start a little test server

*ros2 run udp_example udp_node* starts the node
