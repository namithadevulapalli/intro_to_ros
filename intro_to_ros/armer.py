import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn
from rcl_interfaces.srv import GetParameters

class Armer(Node): 
    
    def __init__(self):
        ''' 
        Initializes node, names it armer, creates a client that can request from the arming service.
        '''
        super().__init__("armer")
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.cli2 = self.create_client(SetMode, '/mavros/set_mode')
        self.get_logger().info("starting service nodes")

        self.control_msg = OverrideRCIn()
        self.control_pub = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            10
        )

    def set_rc_channel(self, channel, pwm):
        self.control_msg.channels = [OverrideRCIn.CHAN_NOCHANGE for _ in range(18)]
        self.control_msg.channels[channel - 1] = max(1100, min(pwm, 1900))
        self.control_pub.publish(self.control_msg)

    def set_rc_channels(self, channels, pwms):
        # self.get_logger().info("maybe moving")
        self.control_msg.channels = [OverrideRCIn.CHAN_NOCHANGE for _ in range(18)]
        for channel, pwm in zip(channels, pwms):
            self.control_msg.channels[channel - 1] = max(1100, min(pwm, 1900))
            self.control_pub.publish(self.control_msg)

    def set_rc_channels_to_neutral(self):
        self.control_msg.channels = [1500 for _ in range(18)]
        self.control_pub.publish(self.control_msg)

    


    	

        
    def send_request(self, bool):
        '''
        takes true if the robot should be armed and false if it should be disarmed
        '''
        self.req = CommandBool.Request()
        self.req.value = bool
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    def set_manual(self):
        self.req2 = SetMode.Request()
        self.req2.base_mode = 192
        self.req2.custom_mode = ""
        self.future2 = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        self.get_logger().info("tried to manual")


    

    

def main(args=None):

    rclpy.init(args=args)

    node = Armer() # creates node with service client
    node.send_request(True) # arms the robot to begin control
    node.set_manual()

    try:
        rclpy.spin(node)
        node.set_rc_channels(
            [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18],
            [1600, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
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