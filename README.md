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
- Copy the colcon workspace from the cloned repository files, into the Home directory. Then
build the workspace to setup the ros navigation package on your Remote PC
    ```
    cp ~/Autonomous-Food-Delivery-System_EG2310/colcon_ws ~/
    colcon build
    ```
- Add the following lines to the .bashrc file on the laptop
   ```
   alias map2base='ros2 run auto_nav map2base'
   alias waypoints='ros2 run auto_nav wayPoints'
   alias start_nav='ros2 run auto_nav moveToGoal'
   ```
## Installing the program on the Turtlebot3's RPi
 - Copy the `hardware_startup` package from the cloned repository files, into the turtlebot3_ws in the RPi.
 After this, build the workspace.
    ```
    scp -r ~/Autonomous-Food-Delivery-System_EG2310/hardware_startup ubuntu@<ip-address-of-RPi>:~/turtlebot3_ws/src
    ssh ubuntu@<ip-address-of-RPi>
    cd turtlebot3_ws
    colcon build
    ```
## Installing the program on the Dispenser bot's ESP32
- INCLUDE STEPS TO DO THE SAME
## Setting up Waypoints for the tables
Now that we are done setting up the softwares for the Delivery bot and Dispenser bot, we can setup the waypoints for the Delivery bot.

**Note**:
- The waypoints currently set are for our particular restaurant layout (refer _restaurant_layout.png_). You will have to **reset the waypoints** before you begin autonomous navigation
- In order to **reuse the waypoints**, the Delivery bot must be turned on from the **same poition** with the same **orientation** all the time. We did this by first fixing the position of the Docking Line (_docking_line.docx_) in the restaurant, which is where you would want your Dispenser Bot to be. Then we placed our Delivery bot on this line before turning it on. In order to make sure we start on the same poisition, we also marked reference points on the Docking Line based on the postion of the Delivery bot.
- An alternative to the above would be to setup the waypoints each time you Restart the Food Delivery System, however, we do not recommmend this.

### Preparation - waypoints
1. Begin by first printing out the Docking Line (_docking_line.docx_) and then joining them together, using tape (DO NOT use dark coloured tape). While joining them together, ensure that the black line is continuous and that there are no white gaps in between. Furthermore, do not use any type of plastic tape on the front as it would cause the Delivery bot's ball castors to slip, resulting in inaccurate docking.  If needed, use white masking tape.
Refer to _example_dockingLine.png_ for an idea on how the end result should look like.
