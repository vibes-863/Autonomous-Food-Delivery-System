# Autonomous-Food-Delivery-System_EG2310 (AY22/23)

We developed a Food Delivery system which consists of a Delivery bot and a Dispenser bot. The Delivery bot, 
collects a soda can from the Dispenser bot and then delivers it to the table selected by the user. The system, 
as of now, can deliver to a maximum of 6 different tables. However, the code has been written such that adding
additional tables is easy. The current navigation system is based on stored waypoints.

Please follow our step by step guide to setup and use our system. This setup guide assumes that you are using
our custom Delivery bot and Dispenser bot. Refer to <include name of report, also add it into the github> for 
a detailed report on our Autonomous Food Delivery System.

## Preparation
1. Follow the instructions here to setup your Remote PC as well as the Turtlebot3. Remember to choose the Foxy tab
   https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Add the following lines to .bashrc of the Remote PC
   ```
   export TURTLEBOT3_MODEL=burger
   alias rteleop='ros2 run turtlebot3_teleop teleop_keyboard'
   alias rslam='ros2 launch turtlebot3_cartographer cartographer.launch.py’
   alias map2base='ros2 run auto_nav map2base'
   alias waypoints='ros2 run auto_nav wayPoints'
   alias start_nav='ros2 run auto_nav moveToGoal'
   ```
3. Add the following lines to .bashrc of the RPi in the Turtlebot3
    ```
    export TURTLEBOT3_MODEL=burger
    alias rosbu='ros2 launch turtlebot3_bringup robot.launch.py'
    alias ir_pub='ros2 run hardware_startup ir_pub'
    alias switch_pub='ros2 run hardware_startup switch_pub'
    ```
  4. Clone this repository into the Remote PC's Home directory
      ```
      git clone https://github.com/vibes-863/Autonomous-Food-Delivery-System_EG2310.git
      ```

## Installing the program on the Remote PC
1. Copy the colcon workspace from the cloned repository files, into the Home directory. Then
build the workspace to setup the ros navigation package on your Remote PC
    ```
    cp ~/Autonomous-Food-Delivery-System_EG2310/colcon_ws ~/
    ```
