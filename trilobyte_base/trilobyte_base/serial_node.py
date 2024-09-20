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
PULSES_PER_METER = 732



class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        # self.subscriber_ = self.create_subscription(String, 'move_command', self.listener_callback, 10)
        self.subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.5)
        self.get_logger().info("serial_node starting . . . ")
    
    def listener_callback(self, msg) -> None:

        serial_msg = self.twist_to_pwm(msg.linear.x, msg.angular.z)
        self.serial_port.write(bytes(serial_msg, 'utf-8'))
        self.get_logger().info(serial_msg)

        time.sleep(0.01)
        # readLine = self.serial_port.readline()
        # stringLine = readLine.decode("utf-8")
        # print(stringLine)
    
    def twist_to_pwm(self, linear_x, angular_z) -> str:

        speed_angle = angular_z
        speed_linear = linear_x

        wheel_left = (speed_linear/WHEEL_RADIUS) - ((speed_angle * WHEEL_SEPARATION)/(2.0 * WHEEL_RADIUS))
        wheel_right = (speed_linear/WHEEL_RADIUS) + ((speed_angle * WHEEL_SEPARATION)/(2.0 * WHEEL_RADIUS))
        pwm_left = int(wheel_left * VEL_TO_PWM)
        pwm_right = int(wheel_right * VEL_TO_PWM)

        # negative velocities are converted to new serial format
        direction_left_wheel = 'f'
        direction_right_wheel = 'f'

        if pwm_left < 0:
            pwm_left = pwm_left * (-1)
            direction_left_wheel = 'b'
        if pwm_right < 0:
            pwm_right = pwm_right * (-1)
            direction_right_wheel = 'b'

        # Limit PWM output 
        if pwm_left > 200:
            pwm_left = 200
        if pwm_right > 200:
            pwm_right = 200
        
        # Pad leading zeroes for string 
        string_left_wheel = str(pwm_left).zfill(3)
        string_right_wheel = str(pwm_right).zfill(3)

        return f"<{direction_left_wheel}{string_left_wheel}{direction_right_wheel}{string_right_wheel}>"




def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()