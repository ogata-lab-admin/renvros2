#!/usr/bin/python



import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
import renv_device
import renvros2
from renvros2.renv_node import RenvNode
import logging
from logging import getLogger


import msg_parser.parser2

host = "192.168.170.237:8080"
typeId = "RENVROS.TEST.DEVICE"
name   = "renvros2-tester"
version = "1.0.0"
device_uuid = None
deviceName = "DEVICE"

class VelocityPublisher(RenvNode):
    def __init__(self):
        super().__init__('velocity_send_device')
        #self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        logger = getLogger(__name__)
        logging.basicConfig(filename='example.log',level=logging.INFO)

        self.init_renv(typeId, name, version, device_uuid, deviceName, logger)

        pub = self.create_renv_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.connect(host)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    pub = VelocityPublisher()
    renvros2.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
