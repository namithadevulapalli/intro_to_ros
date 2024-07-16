import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")

        self.battery_state = None
        self.imu_state = None

        self.battery_subscriber = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        )
        self.battery_subscriber

        self.imu_subscriber = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback, 
            qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        )
        self.imu_subscriber

    

        

        self.volt_checker = self.create_timer(
            5.0, self.check_voltage
        )

        self.get_logger().info("starting subscriber nodes")

    def battery_callback(self, msg):
        self.battery_state = msg

    def imu_callback(self, msg):
        self.imu_state = msg

    def check_voltage(self):
        self.get_logger().info(str(self.battery_state.voltage))
        if self.battery_state.voltage < 3.0:
            self.get_logger().info("battery voltage unsafe")


def main(args = None):
    rclpy.init(args=args)
    node = SensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
