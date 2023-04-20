import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


# publisher to publish whether to turn right, turn left, go straight, or stop, based on the IR sensors. Used for docking
class IR_State_Publisher(Node):

    def __init__(self):
        super().__init__('ir_state')
        self.publisher_ = self.create_publisher(String, 'ir_state', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        GPIO.setmode(GPIO.BCM)
        self.leftIR=27
        self.rightIR=22
        GPIO.setup(self.leftIR,GPIO.IN)
        GPIO.setup(self.rightIR,GPIO.IN)


    def timer_callback(self):
        msg = String()
        if ((GPIO.input(self.leftIR) == 1) and (GPIO.input(self.rightIR) == 1)):
            msg.data = 's' # s stands for stop
        elif ((GPIO.input(self.leftIR) == 0) and (GPIO.input(self.rightIR) == 1)):
            msg.data = 'r' # r stands for right
        elif ((GPIO.input(self.leftIR) == 1) and (GPIO.input(self.rightIR) == 0)):
            msg.data = 'l' # l stands for left
        elif ((GPIO.input(self.leftIR) == 0) and (GPIO.input(self.rightIR) == 0)):
            msg.data = 'f' # f stands for forward
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    ir_state_publisher = IR_State_Publisher()
    try:
        rclpy.spin(ir_state_publisher)
    except KeyboardInterrupt:

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

      ir_state_publisher.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
