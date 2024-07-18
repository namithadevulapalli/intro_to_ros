import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class Armer(Node): 
    
    def __init__(self):
        ''' 
        Initializes node, names it armer, creates a client that can request from the arming service.
        '''
        super().__init__("armer")
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.get_logger().info("starting service nodes")

        
    def send_request(self, bool):
        '''
        takes true if the robot should be armed and false if it should be disarmed
        '''
        self.req = CommandBool.Request()
        self.req.value = bool
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    

    

def main(args=None):

    rclpy.init(args=args)

    node = Armer() # creates node with service client
    node.send_request(True) # arms the robot to begin control

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.send_request(False) # disarms the robot before killing the node
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()


# /mavros/cmd/arming, mavros_msgs/srv/CommandBool