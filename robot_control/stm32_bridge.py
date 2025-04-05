import rclpy
from rclpy.node import Node
# Float32MultiArray
from std_msgs.msg import Float32MultiArray
import spidev
import struct

class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32bridge')
        self.get_logger().info('STM32 Bridge Node has been started')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_commands',
            self.motor_commands_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'encoder_values',
            10)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 100000
    
    def motor_commands_callback(self, msg):
        self.get_logger().info(f'Received motor commands: {msg.data}')

        tx_bytes = struct.pack('<{}f'.format(len(msg.data)), *msg.data)
        rx_list = self.spi.xfer2(list(tx_bytes))
        rx_bytes = bytes(rx_list)
        rx_values = struct.unpack('<{}f'.format(msg.data), rx_bytes)
       
        self.get_logger().info(f'Received encoder values: {rx_values}')

        # publish the encoder values
        msg = Float32MultiArray()
        msg.data = rx_values
        self.publisher.publish(msg)
        self.get_logger().info(f'Published encoder values: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    stm32_bridge = Stm32Bridge()

    rclpy.spin(stm32_bridge)

    stm32_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()