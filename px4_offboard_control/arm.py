import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class UAVControlNode(Node):
    def __init__(self):
        super().__init__('uav_control_node')

        # Define the service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Ensure that the service clients are available before proceeding
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')
        
        # Call the services
        self.arm_uav()

    def arm_uav(self):
        req = CommandBool.Request()
        req.value = True
        self.future_arm = self.arming_client.call_async(req)
        self.future_arm.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arming request successful')
            else:
                self.get_logger().info('Arming request failed')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    uav_control_node = UAVControlNode()
    rclpy.spin(uav_control_node)
    uav_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
