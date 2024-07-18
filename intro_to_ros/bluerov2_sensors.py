import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")

        self.battery_state = None
        self.imu_state = None

        # battery sensor subscriber
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

        # imu data subscriber
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

        # differential pressure subscriber
        self.pressure_diff_subscriber = self.create_subscription(
            FluidPressure,
            "mavros/imu/diff_pressure",
            self.pressure_diff_callback,
            qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        )

        self.pressure_diff_subscriber

        # static pressure subscriber
        self.pressure_static_subscriber = self.create_subscription(
            FluidPressure,
            "mavros/imu/static_pressure",
            self.pressure_static_callback,
            qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        )
        self.pressure_static_subscriber
        
        # timed volt checks
        self.volt_checker = self.create_timer(
            5.0, self.check_voltage
        )

        self.imu_data_checker = self.create_timer(
            5.0, self.print_imu_data
        )

        self.get_logger().info("starting subscriber nodes")

    def pressure_diff_callback(self, msg):
        '''takes sensor data for differential pressure and returns fluid pressure and variance'''
        self.get_logger().info("diff fluid pressure: " + str(msg.fluid_pressure))
        self.get_logger().info("diff variance: " + str(msg.variance))
        

    def pressure_static_callback(self, msg):
        '''takes sensor data for static pressure and returns fluid pressure and variance'''
        self.get_logger().info("static fluid pressure: " + str(msg.fluid_pressure))
        self.get_logger().info("static variance: " + str(msg.variance))
    
    def battery_callback(self, msg):
        '''stores battery information in class variable'''
        self.battery_state = msg

    def imu_callback(self, msg):
        '''stores IMU information in class variable'''
        self.imu_state = msg

    def print_imu_data(self):
        '''prints IMU data'''
        self.get_logger().info(str(self.imu_state))

    def check_voltage(self):
        '''prints current voltage of battery and prints warning if it goes below 12V'''
        self.get_logger().info("current voltage: " + str(self.battery_state.voltage))

        if self.battery_state.voltage < 12.0:
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
