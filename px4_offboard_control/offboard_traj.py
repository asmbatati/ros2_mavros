import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.clock import Clock

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, PositionTarget

from .trajectories import Circle3D, Infinity3D

import numpy as np
import math
import time

class UAVOffboardControl(Node):
    def __init__(self):
        super().__init__('uav_offboard_control')

        self.declare_parameter('system_id', 1)
        self.sys_id_ = self.get_parameter('system_id').get_parameter_value().integer_value

        self.declare_parameter('radius', 5.0)
        self.radius_ = self.get_parameter('radius').get_parameter_value().double_value

        self.declare_parameter('omega', 0.5)
        self.omega_ = self.get_parameter('omega').get_parameter_value().double_value

        self.declare_parameter('trajectory_type', 'circle')
        self.trajectory_type_ = self.get_parameter('trajectory_type').get_parameter_value().string_value

        self.declare_parameter('normal_vector', [0., 0., 1.])
        self.normal_vector_ = self.get_parameter('normal_vector').get_parameter_value().double_array_value

        self.declare_parameter('center', [0., 0., 1.])
        self.center_ = self.get_parameter('center').get_parameter_value().double_array_value

        # Initialize the trajectory generator based on the selected type
        if self.trajectory_type_== 'circle':
            self.trajectory_generator_ = Circle3D(np.array(self.normal_vector_), np.array(self.center_), radius=self.radius_, omega=self.omega_)
        elif self.trajectory_type_ == 'infty':
            self.trajectory_generator_ = Infinity3D(np.array(self.normal_vector_), np.array(self.center_), radius=self.radius_, omega=self.omega_)
        else:
            raise ValueError("Invalid trajectory type. Supported types are 'circle', 'infty'.")

        self.odom_ = Odometry() # latest odom

        # self.arming_client = self.create_client(CommandBool, '/x500/mavros/cmd/arming')
        # self.set_mode_client = self.create_client(SetMode, '/x500/mavros/set_mode')
        # self.setopint_pub_ = self.create_publisher(PoseStamped, '/x500/mavros/setpoint_position/local', QoSProfile(depth=10))

        self.vehicle_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/vehicle_path', 10)
        self.setpoint_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/setpoint_path', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.setopint_pub_ = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', qos_profile_sensor_data)

        timer_period = 0.02  # seconds
        # self.cmd_timer_ = self.create_timer(timer_period, self.cmdloopCallback)

        self.offboard_setpoint_counter_ = 0

        self.is_armed_ = True
        self.dt_ = timer_period

        self.vehicle_path_msg_ = Path()
        self.setpoint_path_msg_ = Path()

        # Wait for service servers to be available
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()

        self.arm_vehicle()
        self.takeoff_sequence()

    def arm_vehicle(self):
        arming_request = CommandBool.Request()
        arming_request.value = True
        future = self.arming_client.call_async(arming_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Vehicle armed successfully')
        else:
            self.get_logger().error('Vehicle arming failed')

    def takeoff_sequence(self):
        t_now = Clock().now()
        point = self.trajectory_generator_.generate_trajectory_setpoint(t_now.nanoseconds / 1000/1000/1000)

        setpoint_msg = PositionTarget()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = self.odom_.header.frame_id
        setpoint_msg.coordinate_frame= PositionTarget.FRAME_LOCAL_NED
        setpoint_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ+ PositionTarget.IGNORE_YAW_RATE
        setpoint_msg.position.x = point[0]
        setpoint_msg.position.y = point[1]
        setpoint_msg.position.z = point[2]
        # yaw  = atan(d_y, d_x)
        yaw = np.arctan2(point[1] - self.odom_.pose.pose.position.y, point[0] - self.odom_.pose.pose.position.x)
        setpoint_msg.yaw = yaw

        self.setopint_pub_.publish(setpoint_msg)
        
         # Publish time history of the vehicle path
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header = self.odom_.header
        vehicle_pose_msg.pose.position = self.odom_.pose.pose.position
        vehicle_pose_msg.pose.orientation = self.odom_.pose.pose.orientation
        self.vehicle_path_msg_.header = self.odom_.header
        self.vehicle_path_msg_.poses.append(vehicle_pose_msg)
        if (len(self.vehicle_path_msg_.poses) > 500):
            self.vehicle_path_msg_.poses.pop(0)

        self.vehicle_path_pub_.publish(self.vehicle_path_msg_)

        # Publish time history of the vehicle path
        setpoint_pose_msg = PoseStamped()
        setpoint_pose_msg.header.frame_id = self.odom_.header.frame_id
        setpoint_pose_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_pose_msg.pose.position.x = point[0]
        setpoint_pose_msg.pose.position.y = point[1]
        setpoint_pose_msg.pose.position.z = point[2]
        self.setpoint_path_msg_.header = setpoint_pose_msg.header
        self.setpoint_path_msg_.poses.append(setpoint_pose_msg)
        if (len(self.setpoint_path_msg_.poses) > 500):
            self.setpoint_path_msg_.poses.pop(0)

        self.setpoint_path_pub_.publish(self.setpoint_path_msg_)
        self.get_logger().info('Published Path successfully')

        self.set_offboard_mode()

    def set_offboard_mode(self):
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
        else:
            self.get_logger().error('Failed to set Offboard mode')


def main(args=None):
    rclpy.init(args=args)
    uav_control = UAVOffboardControl()
    rclpy.spin(uav_control)
    uav_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
