#!/usr/bin/python3

import rclpy
import threading
import time
import math
import tf_transformations
import numpy as np

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from robot_control.robot import robot
from robot_control.can_setup import CAN_setting


class controller(Node):
    def __init__(self):
        super().__init__('omni_controller')
        self.get_logger().info('start node')

        chassis_length  = 1.004
        chassis_width   = 0.484
        wheel_thickness = 0.07

        wheel_offset_x = (chassis_length - (0.0795 * 2)) * 0.5
        wheel_offset_y = (chassis_width + wheel_thickness) * 0.5

        self.GEOMETRI_ROBOT = wheel_offset_x + wheel_offset_y
        self.declare_parameter('mode_orientation', value='odometry')
        self.declare_parameter('wheel_diameter', value=0.1016)
        
        self.create_subscription(Imu, '/imu', self.imu_feedback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Twist, '/omni_cont/cmd_vel_unstamped', self.apply_velocity, qos_profile=qos_profile_system_default)
        self.create_subscription(String, '/rset', self.onReset, qos_profile=qos_profile_system_default)
        self.joints_publisher = self.create_publisher(JointState, '/joint_states', qos_profile=qos_profile_sensor_data)
        self.odom_publisher   = self.create_publisher(Odometry, '/odom', qos_profile=qos_profile_services_default)
        self.odom_broadcaster = TransformBroadcaster(self, qos_profile_services_default)

        self.motor_vel      = np.zeros(4).astype(int)
        self.rotation       = Quaternion()
        self.motor_position = np.zeros(4)

        self.position          = np.zeros(3).astype(float)
        self.can_setup         = CAN_setting()

        self.robot = robot(self.GEOMETRI_ROBOT, self.get_parameter('wheel_diameter').value * 0.5)

        threading.Thread(target=self.read_thread_function).start()
    
    def onReset(self, msg: String):
        if msg.data == 'reset':
            self.motor_vel      = np.zeros(4).astype(int)
            self.rotation       = Quaternion()
            self.motor_position = np.zeros(4)
            self.position       = np.zeros(3).astype(float)
    
    def imu_feedback(self, msg:Imu):
        self.rotation = msg.orientation

    def apply_velocity(self, msg: Twist):
        input = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.motor_vel = self.robot.compute_inverse_kinematic(input, com='RPM')

    def get_rotation_in_rad(self, wheel_rotation):
        return wheel_rotation % (2 * math.pi) - math.pi

    def publish_wheels_state(self, new_position):

        for i in range(len(new_position)):
            self.motor_position[i] += new_position[i]

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_back_left_joint', 'wheel_back_right_joint']
        joint_states.position = [
            self.get_rotation_in_rad(self.motor_position[0]),
            self.get_rotation_in_rad(self.motor_position[1]),
            self.get_rotation_in_rad(self.motor_position[2]),
            self.get_rotation_in_rad(self.motor_position[3])]
        self.joints_publisher.publish(joint_states)

    def publish_odom(self, new_position):

        self.position[0] += new_position[0] * math.cos(self.position[2]) - new_position[1] * math.sin(self.position[2]) 
        self.position[1] += new_position[0] * math.sin(self.position[2]) + new_position[1] * math.cos(self.position[2])
        self.position[2] += new_position[2] 

        tf_quate = tf_transformations.quaternion_from_euler(0, 0, self.position[2])
        rotation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])

        odom_transform = TransformStamped()
        odom_transform.header.stamp            = self.get_clock().now().to_msg()
        odom_transform.header.frame_id         = 'odom'
        odom_transform.child_frame_id          = 'base_footprint'
        odom_transform.transform.translation.x = self.position[0]
        odom_transform.transform.translation.y = self.position[1]
        odom_transform.transform.translation.z = 0.0
        if self.get_parameter('mode_orientation').value == 'odometry':
            odom_transform.transform.rotation  = rotation
        elif self.get_parameter('mode_orientation').value == 'IMU':
            odom_transform.transform.rotation  = self.rotation
        self.odom_broadcaster.sendTransform(odom_transform)

        odometry = Odometry()
        odometry.header.stamp          = self.get_clock().now().to_msg()
        odometry.header.frame_id       = "odom"
        odometry.child_frame_id        = "base_footprint"
        odometry.pose.pose.position.x  = self.position[0]
        odometry.pose.pose.position.y  = self.position[1]
        odometry.pose.pose.position.z  = 0.0
        if self.get_parameter('mode_orientation').value == 'odometry':
            odometry.pose.pose.orientation  = rotation
        elif self.get_parameter('mode_orientation').value == 'IMU':
            odometry.pose.pose.orientation  = self.rotation
        odometry.twist.twist.linear.x  = new_position[0]
        odometry.twist.twist.linear.y  = new_position[1]
        odometry.twist.twist.angular.z = new_position[2]
        self.odom_publisher.publish(odometry)

    def read_thread_function(self):
        while True:
            self.can_setup.send_data_can(self.motor_vel)
            self.publish_odom(self.robot.compute_forward_kinematic(self.can_setup.read_motor_position()))
            self.publish_wheels_state(self.can_setup.read_motor_position())
            

def main(args=None):
    rclpy.init(args=args)
    drive_controller = controller()
    try:
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
        pass
    except BaseException:
        print('Stopped by exception')
        raise
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
