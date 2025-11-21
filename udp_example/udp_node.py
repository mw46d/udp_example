#!/usr/bin/env python3

import json
import socket
import sys
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UDP_Node(Node):
    def __init__(self):
        super().__init__('UDP_Node')

        self.declare_parameters(namespace = '', parameters = [
                ('server', 'localhost'),
                ('port', 4242),
        ])

        self.server_address = (
            self.get_parameter('server').get_parameter_value().string_value,
            self.get_parameter('port').get_parameter_value().integer_value
        )

        # Create a UDP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # The publisher
        self.range_pub = self.create_publisher(Range, '/my_range', 5)

        # main loop timer
        timer_period = 1.0 / 10.0  # 10Hz
        self.timer = self.create_timer(timer_period, self.spin_cb)

        self.get_logger().info("Ready.")

    def spin_cb(self):
        if self.client_socket == None:
            # Reopen the socket
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            message = 'Hello, server!'
            self.client_socket.sendto(message.encode(), self.server_address)

            # Receive the response from the server
            data, address = self.client_socket.recvfrom(1024)
        except socket.error as e:
            self.get_logger().error('Socket error:', e)
            self.client_socket.close()
            self.client_socket = None

        if data != None:
            response = data.decode()
            self.get_logger().info('Received response from server: {}'.format(response))
            reading = json.loads(response)

            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = 'my_sensor_link'

            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = reading['angle']
            range_msg.min_range = reading['min']
            range_msg.max_range = reading['max']
            range_msg.range = reading['value']

            self.range_pub.publish(range_msg)

    def shutdown(self):
        if self.client_socket != None:
            self.client_socket.close()
            self.client_socket = None

def main(args = None):

    if args == None:
        args = sys.argv

    rclpy.init(args = args)

    exit_code = 0

    try:
        node = UDP_Node() 
    except:
        exit_code = 1
        print(traceback.format_exc())
    else:
        try:
            rclpy.spin(node)
        except:
            print(traceback.format_exc())
            node.shutdown()
            node.destroy_node()

    try:
        rclpy.shutdown()
    except:
        pass

    sys.exit(exit_code)

if __name__ == '__main__':
    main(sys.argv)

