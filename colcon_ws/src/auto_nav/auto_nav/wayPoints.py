# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# adapted from https://github.com/Shashika007/teleop_twist_keyboard_ros2/blob/foxy/teleop_twist_keyboard_trio/teleop_keyboard.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose #I added this to subscribe to map2base
import math
import cmath
import numpy as np
import json ## I added this to save the coordinates into a file
from std_msgs.msg import String, Bool # I added this to subscribe to ir_state and switch_state
import time

# constants
ROTATE_CHANGE = 0.1
SPEED_CHANGE = 0.05


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


# function to check if keyboard input is a number as
# isnumeric does not handle negative numbers
def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


# class for moving and rotating robot
class Mover(Node):
    def __init__(self):
        super().__init__('wayPoints')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')

        ## create subscription to get location values from map2base
        self.map2base_subscription = self.create_subscription(
            Pose,
            '/map2base',
            self.map2base_callback,
            10)
        self.map2base_subscription # prevent unused variable warning
        self.x_coordinate = 0 # x-coordinate of the bot in the map
        self.y_coodinate = 0 # y-coordinate of the bot in the map
        self.roll= 0 # roll of the bot in the map (not needed)
        self.pitch= 0 # pitch of the bot in the map (not needed)
        self.yaw = 0 # yaw of the bot in the map (needed)

        # create subscription to get IR sensor data from /ir_state
        self.ir_state_subscription = self.create_subscription(
            String,
            'ir_state',
            self.ir_state_callback,
            10)
        self.ir_state_subscription # prevent unused variable warning
        # initialize variables
        self.ir_state = '' # IR sensor data from the bot: f = forward,r = right, l = left, s = stop

        # variable to store if line is found
        self.foundLine = False
        # variable to count the number of times 's' is received from the IR sensor while docking
        self.count_stop = 0

    # function to set the class variables using the map2base information
    def map2base_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In map2base_callback')
        orientation_quat_map = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat_map.x, orientation_quat_map.y, orientation_quat_map.z, orientation_quat_map.w)
        position_map = msg.position
        self.x_coordinate = position_map.x
        self.y_coodinate = position_map.y
    
    # function to set class variables using the /ir_state information
    def ir_state_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In ir_state_callback')
        self.ir_state = msg.data

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * SPEED_CHANGE
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
    
    # function to dock the robot
    def dock(self):
        # create Twist object
        twist = Twist()
        # set twist such that it rotates in point
        twist.linear.x = 0.0
        twist.angular.z += 0.3
        # keeps rotating until line is found
        while(self.foundLine == False):
              self.count_stop = 0 # reset count_stop
              rclpy.spin_once(self)
              if(self.ir_state == 'r'):
                    self.get_logger().info('Found line')
                    self.foundLine = True
                    twist.angular.z -= 0.3
                    self.publisher_.publish(twist)
                    time.sleep(1)
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    break
              else:
                    self.publisher_.publish(twist)
        
        # now that we have the line in between the ir sensors, we will now start the docking
        self.get_logger().info('Docking...')
        # How the code works is that the robot will keep following the line until it reaches the end of the line.
        # It will only stop completely if the ir_state becomes 's' twice.
        # Hence, the loop will only stop if count_stop becomes 2
        while(self.count_stop != 2):
            rclpy.spin_once(self)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # if the right ir sensor detects a line, then the robot will turn right
            if(self.ir_state == 'r'):
                twist.linear.x = 0.0
                twist.angular.z -= ROTATE_CHANGE
                self.publisher_.publish(twist)
            # if the left ir sensor detects a line, then the robot will turn left
            elif(self.ir_state == 'l'):
                twist.linear.x = 0.0
                twist.angular.z += ROTATE_CHANGE
                self.publisher_.publish(twist)
            # if both ir sensors dont detect a line, then the robot will move forward
            elif(self.ir_state == 'f'):
                twist.linear.x += -(SPEED_CHANGE + 0.01)
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            # if both ir sensors detect a line and the count_stop = 0, then the robot will stop
            elif(self.ir_state == 's' and self.count_stop == 0):
                self.count_stop += 1
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            # if count_stop = 1 then the robot will wait for 2 seconds and check if the ir_state is still 's'
            # if it is, then the robot will stop completely
            # if it is not, then the robot will restart the docking process
            # this is to prevent the robot from stopping when it approaches the line at 90 degrees
            elif(self.count_stop == 1):
                twist.linear.x += -(SPEED_CHANGE + 0.01)
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info('waiting for 2 seconds')
                time.sleep(0.75) # sleep in seconds
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
                rclpy.spin_once(self)
                print(self.ir_state)
                if(self.ir_state == 'f' or self.ir_state == 'l' or self.ir_state == 'r'):
                    self.count_stop = 0
                    self.foundLine = False # reset foundLine
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.get_logger().info('Robot was at 90 degrees at line. Therefore restarted docking.')
                    self.dock()
                elif(self.ir_state == 's'):
                    self.count_stop += 1
                    self.foundLine = False # reset foundLine
        
        # resets foundLine and count_stop to initial values
        self.foundLine = False
        self.count_stop = 0
        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    # function to store the waypoints into a json file
    def storeWaypoint(self):

        rclpy.spin_once(self)
        angle = math.degrees(self.yaw)
        self.get_logger().info('x coordinate: %f y coordinate: %f current yaw: %f' %(self.x_coordinate, self.y_coodinate, angle))
        data = dict()

        try:
            while True:
                # get keyboard input
                point_number = str(input("Enter Point number to which you want to save the data into (Press 'c' to cancel)"))

                if point_number == 'c':
                    # cancel save operation
                    break
                else:
                    with open('/home/vaibhav/colcon_ws/src/auto_nav/auto_nav/wayPointsData.json') as f:
                        data = json.load(f)
                    data['point' + point_number]['x_cord'] = self.x_coordinate
                    data['point' + point_number]['y_cord'] = self.y_coodinate
                    data['point' + point_number]['orientation'] = angle

                    with open('/home/vaibhav/colcon_ws/src/auto_nav/auto_nav/wayPointsData.json', 'w') as f:
                        json.dump(data, f, indent=2)
                    
                    break
        except Exception as e:
            print(e)

    # function to read keyboard input
    def readKey(self):
        twist = Twist()
        try:
            while True:
                # get keyboard input
                cmd_char = str(input("Keys w/x/a/d -/+int s | o - save, p - park: "))
        
                # use our own function isnumber as isnumeric 
                # does not handle negative numbers
                if isnumber(cmd_char):
                    # rotate by specified angle
                    self.rotatebot(int(cmd_char))
                else:
                    # check which key was entered
                    if cmd_char == 's':
                        # stop moving
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    elif cmd_char == 'w':
                        # move forward
                        twist.linear.x += SPEED_CHANGE
                        #twist.angular.z = 0.0
                    elif cmd_char == 'x':
                        # move backward
                        twist.linear.x -= SPEED_CHANGE
                        #twist.angular.z = 0.0
                    elif cmd_char == 'a':
                        # turn counter-clockwise
                        #twist.linear.x = 0.0
                        twist.angular.z += ROTATE_CHANGE
                    elif cmd_char == 'd':
                        # turn clockwise
                        #twist.linear.x = 0.0
                        twist.angular.z -= ROTATE_CHANGE
                    elif cmd_char == 'o': # to save the map values
                        #save the coordinate
                        self.storeWaypoint()
                    elif cmd_char == 'p': # to dock
                        #dock the robot
                        self.dock()
                        
                    # start the movement
                    self.publisher_.publish(twist)
    
        except Exception as e:
            print(e)
            
		# Ctrl-c detected
        finally:
        	# stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()    
    mover.readKey()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()