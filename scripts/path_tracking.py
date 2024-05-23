#!/usr/bin/python3

import rclpy
import math
import numpy as np
import tf_transformations

from geometry_msgs.msg import PoseStamped, Twist, Quaternion, Point
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from visualization_msgs.msg import Marker

class node_maker(Node):

    plan_d = [PoseStamped()]

    step_ = 500
    last_pos_angl = np.zeros(2)
    last_position = np.zeros(2)

    def __init__(self):
        super().__init__('path_tracking')
        self.get_logger().info('start node')

        self.declare_parameter('orientation', value='fixed')

        self.create_subscription(PoseStamped, '/goal_pose', self.onClick_points, qos_profile=qos_profile_system_default)
        self.plan__publisher = self.create_publisher(Path, '/plan', qos_profile=qos_profile_system_default)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)
        self.mark__publisher = self.create_publisher(Marker, '/marker', qos_profile=qos_profile_system_default)

        self.marker_setting()

    def onClick_points(self, msg: PoseStamped):
        line_point = Point()
        line_point.x = msg.pose.position.x
        line_point.y = msg.pose.position.y
        self.marker.points.append(line_point)
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.mark__publisher.publish(self.marker)

        data_send = Path()
        data_send.header.frame_id = 'odom'
        for i in range(self.step_):
            data = PoseStamped()
            if self.get_parameter('orientation').value == 'fixed':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]

            elif self.get_parameter('orientation').value == 'follow_line':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                angle = math.atan2((msg.pose.position.y - self.last_position[1]) - self.last_pos_angl[0], (msg.pose.position.x - self.last_position[0]) - self.last_pos_angl[1])
                tf_quate = tf_transformations.quaternion_from_euler(0, 0, angle)
                data.pose.orientation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])

            elif self.get_parameter('orientation').value == 'setpoint':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                data.pose.orientation = msg.pose.orientation

            elif self.get_parameter('orientation').value == 'follow_line_&_setpoint':
                if i < self.step_ -1:
                    data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                    data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                    angle = math.atan2((msg.pose.position.y - self.last_position[1]) - self.last_pos_angl[0], (msg.pose.position.x - self.last_position[0]) - self.last_pos_angl[1])
                    tf_quate = tf_transformations.quaternion_from_euler(0, 0, angle)
                    data.pose.orientation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])
                else:
                    data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                    data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                    data.pose.orientation = msg.pose.orientation
            
            data_send.poses.append(data)

        self.plan__publisher.publish(data_send)

        self.last_position[0] = msg.pose.position.x
        self.last_position[1] = msg.pose.position.y
    
    def marker_setting(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.01

        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.points = []
        line_point = Point()
        line_point.x = 0.0
        line_point.y = 0.0
        self.marker.points.append(line_point)


def main(args=None):
    rclpy.init(args=args)
    maker = node_maker()
    try:
        rclpy.spin(maker)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
        pass
    except BaseException:
        print('Stopped by exception')
        raise
    finally:
        maker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
