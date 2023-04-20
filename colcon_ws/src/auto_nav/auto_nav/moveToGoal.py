import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Pose 
import math
import cmath
import numpy as np
import json 
import paho.mqtt.client as mqtt
import socket
from std_msgs.msg import String, Bool 
import time
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

# constants
ROTATE_CHANGE = 0.5 # Defines the Rotation speed. Is changed and then reset for some cases
SPEED_CHANGE = 0.15 # Defines the Speed.
ANGLE_ERROR = 5.0 # Defines the acceptable error in angle while rotating
DIST_ERROR = 0.04 # Defines the acceptable error in distance while moving to a point
ANGLE_CHECK_DISTANCE = 0.2 # Defines the distance after which the angle is checked to see if the robot deviated from its path. Is changes and then rest for some cases
SPEED_REDUCTION_DISTANCE = 0.25 # Defines the distance after which the speed is reduced to avoid overshooting
REDUCED_SPEED_CHANGE = 0.05 # Defines the reduced speed
ROTATION_REDUCTION_ANGLE = 20.0 # Defines the angle after which the rotation speed is reduced to avoid overshooting. Is changed and then reset for some cases
IDEAL_ANGLE = 25 # Defines the angle from 0 to which the unknown table is searched for
# defining the individual tables 'points' based on the wayPointsData.json file
table1 = [1,2] 
table2 = [1,2,3]
table3 = [1,10,4]
table4 = [1,10,5]
table5 = [1,10,6,7]
table6 = [1,2,3,8,9,11]
# initializing table_num to -1
table_num = -1 # table number. Note set to -1 so that it acts as a flag
# defining the table range for searching for the unknown table
table_range =range(-IDEAL_ANGLE,IDEAL_ANGLE+1,1)

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



# class for moving the robot based on coordinates
class Navigate(Node):
    def __init__(self):
        super().__init__('moveToGoal')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        # create subscription to get location values from /map2base
        self.map2base_subscription = self.create_subscription(
            Pose,
            '/map2base',
            self.map2base_callback,
            10)
        self.map2base_subscription # prevent unused variable warning
        # initialize variables
        self.x_coordinate = 0 # x-coordinate of the bot in the map
        self.y_coodinate = 0 # y-coordinate of the bot in the map
        self.roll = 0 # roll of the bot in the map (not needed)
        self.pitch = 0 # pitch of the bot in the map (not needed)
        self.yaw = 0 # yaw of the bot in the map

        # create subscription to get IR sensor data from /ir_state
        self.ir_state_subscription = self.create_subscription(
            String,
            'ir_state',
            self.ir_state_callback,
            10)
        self.ir_state_subscription # prevent unused variable warning
        # initialize variables
        # IR sensor data from the bot: f = forward,r = right, l = left, s = stop
        self.ir_state = '' 
        # variable to store if line is found
        self.foundLine = False
        # variable to count the number of times 's' is received from the IR sensor while docking
        self.count_stop = 0

        # create subscription to get switch sensor data from /switch_state
        self.switch_state_subscription = self.create_subscription(
            Bool,
            'switch_state',
            self.switch_state_callback,
            10)
        self.switch_state_subscription # prevent unused variable warning
        # initialize variables
        self.switch_state = False # switch state: True = pressed, False = not pressed

        # create subscription to get LIDAR data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription # prevent unused variable warning
        # initialize variables
        self.laser_range = np.array([]) # array to store the LIDAR data
        self.min_angle = 0 # angle to which we want to turn the robot
        self.min_index = -1 # index of the minimum distance
        self.min_distance = 0 # distance of the min_index
        self.forward_dist = 0 # distance at 0 index


    # function to set the class variables using the /map2base information
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


    # function to set class variables using the /switch_state information
    def switch_state_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In switch_state_callback')
        self.switch_state = msg.data

    
    # function to set class variables using the LIDAR information
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array of lidar ranges
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan's
        self.laser_range[self.laser_range == 0] = np.nan

        # find the minimum range in the table range
        min_distance = 1000 # distance of the min_index

        # the table range is the range of angles that we are interested in
        for i in table_range:
            distance = self.laser_range[i]
            # if the distance is less than the current minimum distance
            # then update the minimum distance and the index
            if distance < min_distance:
                min_distance = distance
                self.min_index = i
        ## convert the index to an angle based on the turtlebot3
        self.min_angle = math.degrees(self.min_index * msg.angle_increment)
        self.min_distance = self.laser_range[self.min_index]


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
        twist.angular.z = c_change_dir * ROTATE_CHANGE
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
            ##self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
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
    

    # function to move to goal
    def moveToGoal(self, next_x, next_y):
        global ROTATE_CHANGE # access the global variable
        global ANGLE_CHECK_DISTANCE # access the global variable
        global ROTATION_REDUCTION_ANGLE # access the global variable

        # create Point object
        goal = Point()
        # create Twist object
        twist = Twist()

        # the x-coordinate we need to go to
        goal.x = next_x
        # the y-coordinate we need to go to
        goal.y = next_y

        # allow the callback functions to run
        rclpy.spin_once(self)

        # inc_x is the difference in the x-coordinate between goal and current
        inc_x = goal.x - self.x_coordinate
        # inc_y is the difference in y-coordinate between goal and current
        inc_y = goal.y - self.y_coodinate
        # distance_to_goal is the distance between the goal and the current position uses the pythagorean theorem
        distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)

        # angle_to_coordinate uses the atan2 function to compute the angle from x-axis to the coordinate in the counter-clockwise direction
        angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
        # angle_to_turn stores the angle between the robots 0 degree and the coordinate
        angle_to_turn = angle_to_goal - math.degrees(self.yaw)

        temp_rotate_change = ROTATE_CHANGE # temporarily store the ROTATE_CHANGE value as we want to update it.
        # while loop to rotate the bot to face the point
        while abs(angle_to_turn) > ANGLE_ERROR:
            # if the angle to turn is less than ROTATION_REDUCTION_ANGLE, the speed of rotating is reduced in order to prevent overshooting
            if abs(angle_to_turn) < ROTATION_REDUCTION_ANGLE:
                ROTATE_CHANGE = 0.1
                self.rotatebot(angle_to_turn)
                ROTATE_CHANGE = temp_rotate_change
            else:
                ROTATE_CHANGE = temp_rotate_change
                self.rotatebot(angle_to_turn)
            # update the x and y and then recalculate the angle to turn
            rclpy.spin_once(self)
            inc_x = goal.x - self.x_coordinate
            inc_y = goal.y - self.y_coodinate
            distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)  
            angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
            angle_to_turn = angle_to_goal - math.degrees(self.yaw)
        
        # initialize variables
        distance_traveled = 0.0 # distance traveled by the robot
        last_angle_check_distance = 0.0 # distance traveled by the robot when the last angle check was done
        distance_left = distance_to_goal # distance left to travel by the robot

        checked = False # boolean to check if the angle has been checked

        temp_angle_check_distance = ANGLE_CHECK_DISTANCE # temporarily store the ANGLE_CHECK_DISTANCE value as we want to update it
        temp_rotation_reduction_angle = ROTATION_REDUCTION_ANGLE # temporarily store the ROTATION_REDUCTION_ANGLE value as we want to update it
 
        # while loop to move the robot straight until it reaches the goal
        while(distance_to_goal > DIST_ERROR):
            # cleares the twist object to avoid adding values
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # allow the callback functions to run to update the distance to goal
            rclpy.spin_once(self)

            # updates the distance to goal
            inc_x = goal.x - self.x_coordinate
            inc_y = goal.y - self.y_coodinate
            distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)
            # prints the distance to goal
            self.get_logger().info('distance: %f' %(distance_to_goal))
            # updates the distance traveled
            distance_traveled = abs(distance_left - distance_to_goal)
            # moves the robot straight
            # if the distance to goal is less than the SPEED_REDUCTION_DISTANCE, the speed of the bot is reduced in order to prevent overshooting
            if distance_to_goal < SPEED_REDUCTION_DISTANCE:
                ROTATION_REDUCTION_ANGLE = 50.0
                ANGLE_CHECK_DISTANCE = 0.1

                twist.linear.x += REDUCED_SPEED_CHANGE
                twist.angular.z = 0.0
            else:
                ROTATION_REDUCTION_ANGLE = temp_rotation_reduction_angle
                ANGLE_CHECK_DISTANCE = temp_angle_check_distance
                twist.linear.x += SPEED_CHANGE
                twist.angular.z = 0.0
            # publish the twist msg
            self.publisher_.publish(twist)

            # if statement to check if the robot has traveled ANGLE_CHECK_DISTANCE since the last angle check
            if (abs(distance_traveled - last_angle_check_distance) >= ANGLE_CHECK_DISTANCE and distance_to_goal >= DIST_ERROR) or (distance_to_goal < (SPEED_REDUCTION_DISTANCE-0.1) and checked == False):
                # updates the angle to goal and angle to turn
                rclpy.spin_once(self)
                angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
                angle_to_turn = angle_to_goal - math.degrees(self.yaw)

                # to correct the angle if the robot deviated from its path
                while abs(angle_to_turn) > ANGLE_ERROR:
                    # to ensure that the robot at least checks once if the distance to goal is less that SPEED_REDUCTION_DISTANCE
                    if distance_to_goal < SPEED_REDUCTION_DISTANCE:
                        checked = True

                    self.get_logger().info('correcting angle')
                    if abs(angle_to_turn) < ROTATION_REDUCTION_ANGLE:
                        ROTATE_CHANGE = 0.1
                        self.rotatebot(angle_to_turn)
                        ROTATE_CHANGE = temp_rotate_change
                    else:
                        ROTATE_CHANGE = temp_rotate_change
                        self.rotatebot(angle_to_turn)
                    # updates the angle to turn
                    rclpy.spin_once(self)
                    inc_x = goal.x - self.x_coordinate
                    inc_y = goal.y - self.y_coodinate
                    distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)  
                    angle_to_goal = math.degrees(math.atan2(inc_y, inc_x))
                    angle_to_turn = angle_to_goal - math.degrees(self.yaw)              
                # updates the last angle check distance
                last_angle_check_distance = distance_traveled
            # while loop ends here
        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    # function to dock the robot
    def dock(self):
        # create Twist object
        twist = Twist()
        # set twist such that it rotates in point
        twist.linear.x = 0.0
        twist.angular.z += ROTATE_CHANGE
        # keeps rotating until line is found
        while(self.foundLine == False):
              rclpy.spin_once(self)
              if(self.ir_state == 'r'):
                    self.get_logger().info('Found line')
                    self.foundLine = True
                    twist.angular.z -= ROTATE_CHANGE
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
                twist.angular.z -= 0.1
                self.publisher_.publish(twist)
            # if the left ir sensor detects a line, then the robot will turn left
            elif(self.ir_state == 'l'):
                twist.linear.x = 0.0
                twist.angular.z += 0.1
                self.publisher_.publish(twist)
            # if both ir sensors dont detect a line, then the robot will move forward
            elif(self.ir_state == 'f'):
                twist.linear.x += -0.06
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
                twist.linear.x += -0.06
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info('waiting for 2 seconds')
                time.sleep(0.75) # sleep in seconds
                rclpy.spin_once(self)
                rclpy.spin_once(self)
                print(self.ir_state)
                if(self.ir_state == 'f' or self.ir_state == 'r' or self.ir_state == 'l'):
                    self.count_stop = 0
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.get_logger().info('Robot was at 90 degrees at line. Therefore restarted docking.')
                    self.dock()
                elif(self.ir_state == 's'):
                    self.count_stop += 1
                    self.foundLine = False

        # resets foundLine and count_stop to initial values
        self.foundLine = False
        self.count_stop = 0
        # stop moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    # function to find and move to the unknown table
    def findUnknownTable(self):
        global ROTATE_CHANGE # access the global variable
        # allow the callback functions to run to update the laser scan data
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        # turn angle is the angle we need to turn to
        turn_angle = self.min_angle
        # boolean variable to store if we found the unknown table
        unknown_table_found = False
        # create Twist object
        twist = Twist()
        # print the angle to the unknown table
        print (turn_angle)
        temp_rotate_change = ROTATE_CHANGE # temporily store the ROTATE_CHANGE value as we want to update it
        ROTATE_CHANGE = 0.1
        # rotate the robot to the angle of the unknown table
        self.rotatebot(turn_angle)
        ROTATE_CHANGE = temp_rotate_change
        # print that we are facing the unknown table
        self.get_logger().info('Pointing to unknown table')
        # allow the callback functions to run to update the laser scan data
        rclpy.spin_once(self)
        # print the distance to the table
        self.get_logger().info('Distance: %f meters' % (self.forward_dist))
        # moves the robot forward until it reaches the unknown table
        twist.linear.x += 0.05
        twist.angular.z = 0.0
        # publish the twist message
        self.publisher_.publish(twist)

        # while the robot is not at the unknown table
        while unknown_table_found == False:
            rclpy.spin_once(self)
            # stores the values of any degree at which the distance is less than 0.40m
            lri = (self.laser_range[table_range]<float(0.40)).nonzero()

            # if the list is not empty then the robot is at the unknown table
            if(len(lri[0])>0):
                # stop moving
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                # stop the while loop
                unknown_table_found = True


    # function to use waypoints to navigate to individual Tables
    def moveToTable(self, table_num):
        # loads the coordinate data from the wayPointsData.json file into the variable data, as a dictionary
        with open('/home/vaibhav/colcon_ws/src/auto_nav/auto_nav/wayPointsData.json') as f:
            data = json.load(f) 
        # returns the array of the table mentioned by table_num
        current_table = globals()[f"table{table_num}"]

        # goes through each point in the array of current_table
        for point_number in current_table:
            # extracts the x-coordinate of the point in the current_table's array
            x_cord = data['point' + str(point_number)]['x_cord']
            # extracts the y-coordinate of the point in the current_table's array
            y_cord = data['point' + str(point_number)]['y_cord']
            # extracts the orientation of the robot at the point in the currect_table's array
            orientation = data['point' + str(point_number)]['orientation'] # Not using as of now

            # calls the function to move the robot to the point
            self.moveToGoal(x_cord,y_cord)
            # prints to the terminal that the point has been reached
            self.get_logger().info('Reached point %d' %(point_number))
        
        # rotates to match saved orientation
        angle_to_goal = orientation
        # angle_to_turn stores the angle between the robots 0 degree and the coordinate
        angle_to_turn = angle_to_goal - math.degrees(self.yaw)
        self.rotatebot(angle_to_turn)

        # if statement to check if the table to go is 6, if yes
        # then the robot will go to the unknown table
        if (table_num == 6):
            self.findUnknownTable()
            #print("rotating 180 degrees")
            #self.rotatebot(180)
        time.sleep(1) # hopefully it helps with the sometimes not turning thing
        # if statement to check if the table to go is 6, if yes
        # then the robot will rotate 180 degrees
        if (table_num == 6):
            print("rotating 180 degrees")
            self.rotatebot(179)

        #prints to the terminal that the table has been reached
        self.get_logger().info('Reached table %d' %(table_num))

        # wait for the can to be picked (switch_state to become false)
        rclpy.spin_once(self)
        start_time = time.time()
        while((self.switch_state == True) and ((time.time() - start_time) < 8)):
            rclpy.spin_once(self)
        # prints to the terminal that the can has been picked
        self.get_logger().info('Can picked')
        time.sleep(2) # wait for some time after can is picked

        # goes through each point in the array of current_table in reverse order
        for point_number in list(reversed(current_table))[1:]:
            # extracts the x-coordinate of the point in the current_table's array
            x_cord = data['point' + str(point_number)]['x_cord']
            # extracts the y-coordinate of the point in the current_table's array
            y_cord = data['point' + str(point_number)]['y_cord']
            # extracts the orientation of the robot at the point in the currect_table's array
            #orientation = data['point' + point_number]['orientation']  Not using as of now

            # calls the function to move the robot to the point
            self.moveToGoal(x_cord,y_cord)
            # prints to the terminal that the point has been reached
            self.get_logger().info('Reached point %d' %(point_number))
        
        # rotates to match saved orientation in this case it is always coordinate of point 1
        angle_to_goal = orientation
        # angle_to_turn stores the angle between the robots 0 degree and the coordinate
        angle_to_turn = angle_to_goal - math.degrees(self.yaw)
        #self.rotatebot(1)
        self.rotatebot(angle_to_turn)
        
        # prints to the terminal that robot has reached the docking point
        self.get_logger().info('Reached docking point')

        # initiates docking
        self.dock()
        # once docked, prints to the terminal that the robot has docked
        self.get_logger().info('Docked!')



# function to store the msg sent by the esp32 into the global variable table_num
def on_table_num(client, userdata, msg):
    global table_num 
    table_num = int(msg.payload.decode('utf-8'))
    print(table_num) # added cuz without this IT WONT WORK


# main function
def main(args=None):
     global table_num
     rclpy.init(args=args)
     # to get ip address of the laptop
     my_ip = socket.gethostbyname(socket.gethostname())

     # to connect to the mqtt broker
     client = mqtt.Client("Turtlebot")
     client.message_callback_add('esp32/output', on_table_num)
     client.connect(my_ip, 1883)
     client.loop_start()
     client.subscribe("esp32/output")

     # to start the navigation based on the table number esp32 sends. The code runs forever.
     navigation = Navigate()
     while True:
         #print (table_num)
         if(table_num != -1):
             # send message to esp32 to tell it that the robot has un-docked and is moving to the table
             client.publish("esp32/input", "0")
             navigation.moveToTable(table_num)
             #navigation.findUnknownTable()
             table_num = -1
             # send message back to esp32 to tell it that the robot has docked
             client.publish("esp32/input", "1")
         
         pass
     
     navigation.destroy_node()
     rclpy.shutdown()
    
        

if __name__ == '__main__':
    main()