import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import time

class UAVTakeoffNode(Node):
    def __init__(self):
        super().__init__('uav_takeoff_node')
        self.declare_parameter('takeoff_height', 5.0)
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        
        self.pose_pub = self.create_publisher(PoseStamped, '/target/mavros/setpoint_position/local', 10)
        
        self.arm_client = self.create_client(CommandBool, '/target/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/target/mavros/cmd/arming service not available, waiting again...')
        
        self.timer = self.create_timer(1, self.takeoff)

    def takeoff(self):
        self.timer.cancel()  # Ensure this method is called only once
        
        takeoff_pose = PoseStamped()
        takeoff_pose.header = Header()
        takeoff_pose.header.stamp = self.get_clock().now().to_msg()
        takeoff_pose.header.frame_id = "map"
        
        takeoff_pose.pose.position = Point(x=0.0, y=0.0, z=self.takeoff_height)
        takeoff_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.get_logger().info('Publishing takeoff position')
        self.pose_pub.publish(takeoff_pose)

        time.sleep(10)
        
        self.get_logger().info('Requesting to arm the UAV')
        arm_request = CommandBool.Request()
        arm_request.value = True
        
        future = self.arm_client.call_async(arm_request)
        future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('UAV arming successful')
            else:
                self.get_logger().info('UAV arming failed')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = UAVTakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
