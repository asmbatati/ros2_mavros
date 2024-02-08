import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL

class UAVTakeoffNode(Node):
    """
    UAVTakeoffNode is a ROS 2 node for arming a UAV and commanding it to take off.

    This node communicates with MAVROS to send arming and takeoff commands to a UAV. It uses
    the '/target/mavros/cmd/arming' service for arming the UAV and the '/target/mavros/cmd/takeoff'
    service for initiating the takeoff procedure.
    """
    def __init__(self):
        """
        Initializes the UAVTakeoffNode, creates service clients for arming and takeoff,
        and sends the requests.
        """
        super().__init__('uav_takeoff_node')

        # Define the service clients for arming and takeoff.
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Ensure that the service clients are available before proceeding.    
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/target/mavros/cmd/takeoff service not available, waiting again...')
        
        # Call the service to initiate takeoff.
        self.takeoff_uav()

    def takeoff_uav(self):
        """
        Sends a request to the '/target/mavros/cmd/takeoff' service to take off the UAV.

        The takeoff command includes parameters such as pitch, yaw, latitude, longitude,
        and altitude, specifying the desired takeoff behavior and target altitude.
        """
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 10.0  # Target altitude in meters
        self.future_takeoff = self.takeoff_client.call_async(req)
        self.future_takeoff.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        """
        Callback function for the takeoff service call.

        Logs the outcome of the takeoff request based on the response received.

        :param future: Future object representing the service call result.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff request successful')
            else:
                self.get_logger().info('Takeoff request failed')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    """
    Main function to initialize the ROS 2 rclpy library, create the UAVTakeoffNode,
    and spin the node to keep it alive for processing callbacks.
    """
    rclpy.init(args=args)
    uav_takeoff_node = UAVTakeoffNode()
    rclpy.spin(uav_takeoff_node)
    uav_takeoff_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
