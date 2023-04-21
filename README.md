# Autonomous-Food-Delivery-System_EG2310 (AY22/23)

We developed a Food Delivery system which consists of a Delivery bot and a Dispenser bot. The Delivery bot, 
collects a soda can from the Dispenser bot and then delivers it to the table selected by the user. The system, 
as of now, can deliver to a maximum of 6 different tables. However, the code has been written such that adding
additional tables is easy. The current navigation system is based on stored waypoints.

Please follow our step by step guide to setup and use our system. This setup guide assumes that you are using
our custom Delivery bot and Dispenser bot. Refer to _Hardware-Design-Document.pdf_ for a detailed report on our Autonomous Food Delivery System.

**Note**: This system requires the Delivery bot's RPi, the Dispenser Bot's ESP32, and the Remote PC to be connected to the same network.

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
4. Setup MQTT for communication between the **Remote PC** and the Dispenser bot (ESP32)
- Copy the following commands onto the Linux terminal to install the MQTT broker onto the Remote PC
    ```
    sudo apt update -y
    sudo apt install mosquitto mosquitto-clients -y
    sudo systemctl start mosquitto
    ```
-  Follow the instructions here to download and setup the MQTT library onto the Dispenser bot
   https://github.com/knolleary/pubsubclient/archive/master.zip](https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/)
   
6. Clone this repository into the Remote PC's Home directory
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
- Add the following lines to the .bashrc file on the Remote PC
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
- Download the `esp32code_v3.4` package from the cloned repository files, into your Remote PC
- Open `esp32code_v3.4.ino` with Arudino IDE 2.0.3
- Follow the `INITIALIZATION` steps
- Connect the ESP32 to the Remote PC with a Micro USB cable
- Flash the code onto the ESP32 whilst simultaneously pressing the `BOOT` button on the ESP32   
## Setting up Waypoints for the tables
Now that we are done setting up the softwares for the Delivery bot and Dispenser bot, we can setup the waypoints for the Delivery bot.

**Note**:
- The waypoints currently set are for our particular restaurant layout (refer _restaurant_layout.png_). You will have to **reset the waypoints** before you begin autonomous navigation
- In order to **reuse the waypoints**, the Delivery bot must be turned on from the **same poition** with the same **orientation** all the time. We did this by first fixing the position of the Docking Line (_docking_line.docx_) in the restaurant, which is where you would want your Dispenser Bot to be. Then we placed our Delivery bot on this line before turning it on. In order to make sure we start on the same poisition, we also marked reference points on the Docking Line based on the postion of the Delivery bot.
- An alternative to the above would be to setup the waypoints each time you restart the Food Delivery System, however, we do not recommmend this.
- For our System, we set waypoint 1 as the **Docking Point**. The **Docking Point** is the waypoint at which the Delivery bot begins the **Docking** procedure. Ensure that the **Docking Point** is close enough to **Docking Line** such that if the Delivery bot rotates in poisition at that point, the **Docking Line** will be detected by the **Infrared Sensors**.

### Preparation - waypoints
1. Begin by first printing out the Docking Line (_docking_line.docx_) and then joining them together, using tape (DO NOT use dark coloured tape). While joining them together, ensure that the black line is continuous and that there are no white gaps in between. Furthermore, do not use any type of plastic tape on the front as it would cause the Delivery bot's ball castors to slip, resulting in inaccurate docking.  If needed, use white masking tape.
Refer to _example_dockingLine.png_ for an idea on how the end result should look like.
2. Now decide the position of the Dispenser bot, make sure there is enough space in the front for the docking line to be placed. Then, fix the Docking line with the thick horizontal line starting at the Dispenser bot, refer to _restaurant_layout.png_.
3. Place the Delivery bot on this line, mark a few reference points based on its position (the position will hereafter be refered to as **Initial position**), and then turn on the Delivery bot. These reference points will help when you want to restart the Delivery bot.

### Let's begin setting the waypoints
After completing the preparation, we can begin setting the waypoints. Note that the preparation is a one time thing. If you want to adjust the waypoints later, you only need to follow the following steps.

1. If the Delivery bot is not turned on yet, place the bot on the **Initial position** and then turn it on.
2. On the **Remote PC**, open **4 Terminals** (we recommend installing and using **Terminator** as it is much more convinent when working with multiple terminals)
3. In **Terminal 1**
   ```
   ssh ubuntu@<ip-address-of-RPi>
   rosbu
   ```
   In **Terminal 2**
   ```
   rslam
   ```
   In **Terminal 3**
   ```
   map2base
   ```
   In **Terminal 4**
   ```
   waypoints
   ```
4. Now that all the Terminals are running, you can begin teleoperating the Delivery bot and saving the waypoints, by entering the following commands into **Terminal 4**.
   
   For teleoperating: Enter `w`,`a`,`d`, and `x` to move the Delivery bot Forward, Turn Left, Turn Right, and Backward respectively. Enter `s` to Stop the bot.
   
   For saving the waypoints: Enter `o` and then enter the point number of the waypoint, for example `1`. This will save the waypoint's x-coordinate, y-coordinate, and current orientation, into point 1 in _wayPointsData.json_. 
   
   After saving all the waypoints, verify that they have been updated in _wayPointsData.json_.
5. You have finished setting up the waypoints! You now have two choices:
   - Begin **Autonomous Navigation** - Now that you are done setting up the waypoints, the Autonomous Food Delivery System is almost ready to deliver soda cans to your customers! Do the following using **Terminal 4**:
      - Teleoperate the Delivery bot to the **Docking Point**
      - Once the bot is at the **Docking Point**, enter `p`. This will begin the **Docking** procedure.
      - Once the bot has docked, align the Dispenser bot such that the soda can will roll down into the **container** of the Delivery bot.
      - You can now move onto the next section's (**Let's Start Delivering!**) **Initializing** sub-section.
   - Stop for now - If you choose to stop for time being and then continue later then you can do the following:
      -  Stop all running code by entering `Ctrl + c` on all Terminals
      -  Next, when you want to start **Autonomous Navigation**, you can begin by following the steps from the next section (**Let's Start Delivering!**)

## Let's Start Delivering!
Now that we are done with all the preparation, we can begin delivering soda cans. However before starting, there is a little initialization you need to do.
### Initializing
Follow these steps to get all the software required by the Autonomous Food Delivery System to start:
1. Place the Delivery bot on the **Initial Position** and then turn it on
2. Open **6 Terminals** on the **Remote PC**. Three of these will be used to start up the hardware on the Delivery bot, and the other three will be used to begin the navigation softwares.
3. In **Terminal 1**
   ```
   ssh ubuntu@<ip-address-of-RPi>
   rosbu
   ```
   In **Terminal 2**
   ```
   ssh ubuntu@<ip-address-of-RPi>
   ir_pub
   ```
   In **Terminal 3**
   ```
   ssh ubuntu@<ip-address-of-RPi>
   switch_pub
   ```
   In **Terminal 4**
   ```
   rslam
   ```
   In **Terminal 5**
   ```
   map2base
   ```
   In **Terminal 6**
   ```
   start_nav
   ```
4. If you had carried out Step 1, that is, you began with the Delivery bot at the Docking point, you will need to dock the Delivery bot. 

   To do this, open another Terminal and enter `mosquitto_pub -t "esp32/output" -m "1"`, this will result in the Delivery bot going to Table 1 and then will return and dock. 
   Once docked, remember to align the Dispenser bot such that the soda can will roll down into the **container** of the Delivery bot.
   
With that, you have successfully initialized the Autonomous Food Delivey System!
### Steps to deliver a soda can to a Table
1. Select a destination table by pressing a number 1-6 on the keypad
2. Press '#' to confirm the table number and the Delivery robot will start delivering to said table if docked. Else, the number is added to queue
3. Press "*" to delete selected table number

That's it! You have now successfully setup and run a state of the art Autonomous Food Delivery System
   
## Parameters that affect the operation of the Delivery bot
From lines 16 to 36 of /colcon_ws/src/auto_nav/auto_nav/moveToGoal.py

The explanation of all the parameters are as commented below. These are the values which we found work the best for us in our use case. Please change these values if you require a different behavior of the robot
``` python
# constants
ROTATE_CHANGE = 0.5 # Defines the Rotation speed. Is changed and then reset for some cases
SPEED_CHANGE = 0.15 # Defines the Speed.
ANGLE_ERROR = 5.0 # Defines the acceptable error in angle while rotating
DIST_ERROR = 0.04 # Defines the acceptable error in distance while moving to a point
ANGLE_CHECK_DISTANCE = 0.2 # Defines the distance after which the angle is checked to see if the robot deviated from its path. Is changed and then rest for some cases
SPEED_REDUCTION_DISTANCE = 0.25 # Defines the distance after which the speed is reduced to avoid overshooting
REDUCED_SPEED_CHANGE = 0.05 # Defines the reduced speed
ROTATION_REDUCTION_ANGLE = 20.0 # Defines the angle after which the rotation speed is reduced to avoid overshooting. Is changed and then reset for some cases
IDEAL_ANGLE = 25 # Defines the angle from 0 to which the unknown table is searched for
# defining the individual table's 'waypoints' based on the wayPointsData.json file
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
```
## Other information
- The navigation code (_auto_nav_ package) was initially developed in another repository : https://github.com/Hidiebye11/r2auto_nav.git
- Detailed explanation on the objectives of this system as well as the design of our restaurant can be refered to in the file _final_mission_rubrics.pdf_
