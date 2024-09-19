import rclpy
from rclpy.node import Node
import serial
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

# Physical Robot Configuration
WHEEL_RADIUS = 0.030
WHEEL_SEPARATION = 0.24
VEL_TO_PWM = 278
PULSES_PER_REVOLUTION = 138



class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        # self.subscriber_ = self.create_subscription(String, 'move_command', self.listener_callback, 10)
        self.subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.5)
        self.get_logger().info("serial_node starting . . . ")
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        # self.serial_port.write(bytes(msg.data, 'utf-8'))
        self.twist_to_pwm(msg.linear.x, msg.angular.z)
        time.sleep(0.01)
        readLine = self.serial_port.readline()
        stringLine = readLine.decode("utf-8")
        print(stringLine)
        # self.serial_port.close()
    
    def twist_to_pwm(self, linear_x, angular_z):

        pwm_left, pwm_right = 0, 0

        speed_angle = angular_z
        speed_linear = linear_x

        wheel_left = (speed_linear/WHEEL_RADIUS) - ((speed_angle * WHEEL_SEPARATION)/(2.0 * WHEEL_RADIUS))
        wheel_right = (speed_linear/WHEEL_RADIUS) + ((speed_angle * WHEEL_SEPARATION)/(2.0 * WHEEL_RADIUS))
        pwm_left = wheel_left * VEL_TO_PWM
        pwm_right = wheel_right * VEL_TO_PWM

        # TODO Account for negative PWM for shifting wheel direction and make appropriate calculations
        if pwm_left > 200:
            pwm_left = 200
        if pwm_right > 200:
            pwm_right = 200

        return f"<"




def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()