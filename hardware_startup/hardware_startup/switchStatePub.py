import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


# publisher to publish if limit switch has been pressed or not, to indicate presence of can
class SwitchState_Publisher(Node):

    def __init__(self):
        super().__init__('switch_state')
        self.publisher_ = self.create_publisher(Bool, 'switch_state', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        GPIO.setmode(GPIO.BCM)
        self.switchpin = 17
        GPIO.setup(self.switchpin,GPIO.IN)


    def timer_callback(self):
        msg = Bool()
        msg.data = False
        if (GPIO.input(self.switchpin)) == 1:
            msg.data = True
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    switchstate_publisher = SwitchState_Publisher()
    try:
      rclpy.spin(switchstate_publisher)
    except KeyboardInterrupt:

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

      switchstate_publisher.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
