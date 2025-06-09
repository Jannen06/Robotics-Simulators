# Robotics-Simulators

An investigation of simulators and hands-on

# Webots Simulator

![ROSBOT](/src/webots_sim/doc/figures/webot_simulator.png)

## Installation
In addition to a few ROS dependencies, this package also depends on the following packages provided by webots:
1. [webots env](https://cyberbotics.com/#download) 
2. Other dependencies
```
sudo apt update
sudo apt install libgl1 libglu1-mesa libxi6 libxmu6 libpng16-16 libjpeg8 libsqlite3-0
```
3. Navigate to download folder and install
```
cd ~/Downloads
sudo dpkg -i webots*.deb
```


Assuming you have ROS2 Humble installed and working, you can move forward.



## Usage

To start this simulator and launch the file from repo:

1. clone the git repo
```
git clone git@github.com:H-BRS-SER/testing-and-quality-assurance-nullpointers.git
```
2. Navigate to the package `Simulator` and build.
```
cd ~/testing-and-quality-assurance-nullpointers/src/Simulator/
colcon build
source install/setup.bash
```
3. Launch the simulator

~~~sh
ros2 launch webots_sim rosbot.launch.py
~~~


4. You can then run the node `state_machine` to move the robot around in a free movement and observe obstacle avoidance. 
There is no fixed goal been set. It navigates through the rooms and avoid abosacle in long run. 

~~~sh
ros2 run webots_sim state_machine
~~~


### Moving the robot using a keyboard

The [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) ROS package can be used to simplify controlling the motion of the robot. This requires a one time installation of the `teleop_twist_keyboard` package using the below command:

~~~ sh
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
source /opt/ros/$ROS_DISTRO/setup.bash
~~~

Then start the `teleop_twist_keyboard.py` node using the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` and move the robot as per the instructions displayed on the terminal. Below is an example of the output after running the node:

~~~
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
~~~

