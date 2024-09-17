import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        self.subscriber_ = self.create_subscription(String, 'move_command', self.listener_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.5)
        self.get_logger().info("Testing node . . . ")
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.serial_port.write(bytes('<f100f100>', 'utf-8'))
        time.sleep(0.01)
        readLine = self.serial_port.readline()
        stringLine = readLine.decode("utf-8")
        print(stringLine)
        self.serial_port.close()




def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()