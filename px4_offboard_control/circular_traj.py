import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np
import math
import time

class UAVOffboardControl(Node):
    def __init__(self):
        super().__init__('uav_offboard_control')

        self.arming_client = self.create_client(CommandBool, '/target/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/target/mavros/set_mode')
        self.takeoff_pub = self.create_publisher(PoseStamped, '/target/mavros/setpoint_position/local', QoSProfile(depth=10))

        # Parameters for the takeoff point
        self.takeoff_height = 2.0  # meters
        self.takeoff_rate = 10  # Hz
        self.circle_radius = 5.0  # meters
        self.omega = 0.1  # radians per second

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
        takeoff_point = PoseStamped()
        takeoff_point.header = Header()
        takeoff_point.header.frame_id = "base_link"
        takeoff_point.pose.position.z = self.takeoff_height
        takeoff_point.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        for _ in range(100):  # Publish the takeoff point for 10 seconds at 10 Hz
            self.takeoff_pub.publish(takeoff_point)
            time.sleep(1 / self.takeoff_rate)

        self.set_offboard_mode()

    def set_offboard_mode(self):
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
            self.execute_circle_trajectory()
        else:
            self.get_logger().error('Failed to set Offboard mode')

    def execute_circle_trajectory(self):
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while rclpy.ok():
            current_time = self.get_clock().now().seconds_nanoseconds()[0] - start_time
            x = self.circle_radius * math.cos(self.omega * current_time)
            y = self.circle_radius * math.sin(self.omega * current_time)
            z = self.takeoff_height

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position = Point(x=x, y=y, z=z)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            self.takeoff_pub.publish(pose)
            time.sleep(1 / self.takeoff_rate)

def main(args=None):
    rclpy.init(args=args)
    uav_control = UAVOffboardControl()
    rclpy.spin(uav_control)
    uav_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
