import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')
        self.get_logger().info('Velocity control node')

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.imu_data = {}
    
    def imu_callback(self, msg):
        self.imu_data = {
            "angular_velocity": [ msg.angular_velocity.x, msg.angular_velocity.y,
                                 msg.angular_velocity.z ],
            "linear_acceleration": [ msg.linear_acceleration.x, msg.linear_acceleration.y,
                                    msg.linear_acceleration.z ]
        }
        self.get_logger().info(f'IMU data: {self.imu_data}')
        
def main(args=None):
    rclpy.init(args=args)

    velocity_control_node = VelocityControlNode()

    rclpy.spin(velocity_control_node)

    velocity_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
