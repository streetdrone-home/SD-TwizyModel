# SD Twizy  Vehicle Simulation

Gazebo simulation packages for the SD Twizy vehicle

### Requirements:

##### - Ubuntu 16.04 LTS
##### - ROS Kinetic [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu)
##### - Catkin Command Line Tools [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
##### - Gazebo [ros-kinetic-gazebo-ros-pkgs](http://gazebosim.org/tutorials?tut=ros_installing)  
This model has been tested with Gazebo 7.16. Run `gazebo --version` to make sure you have the correct version installed.  
To update to Gazebo 7.16, do:  
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update && sudo apt upgrade
```
##### - Install dependencies:
```
rosdep install --from-paths src/ --ignore-src -r -y

```

### Setup your workspace:
#### A. Create a catkin workspace:
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```
For more information, visit [create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

#### B. Initialize your catkin workspace:
Navigate to the root of your catkin workspace, if you are not already with `cd ~/catkin_ws`.
Initialize your workspace:
```
catkin init
```

#### C. Clone this repository or copy its contents at your `~/catkin_ws/src` folder of the catkin workspace you just initialized.
#### D. Navigate to your workspace and build the simulation
```
cd ~/catkin_ws
catkin build sd_robot sd_control sd_description sd_control_msgs
```
If you have previously built your workspace with catkin_make:
Either clean your workspace with `catkin clean` and rebuild with `catkin build`
or build the SD Twizy Gazebo packages in isolation with `catkin_make --pkg sd_robot sd_control sd_description`.
After the built has successfully finished, do:
```
source devel/setup.bash
```
#### E. Launch the simulation:
This launches the vehicle model in Gazebo and RViz for visualizing the sensors' output.
```

roslaunch sd_robot sd_twizy_empty.launch

```

<p align="center"> 
<img src="streetdrone_model/sd_docs/imgs/sd.png">
</p>

You might need to update your ignition-math version `sudo apt upgrade libignition-math2`
You might need to update your ignition-math version `sudo apt upgrade libignition-math2`

## Controlling the Robot
### Joystick
The robot supports the generic Linux
[joystick](http://wiki.ros.org/joy) controllers. The `sd_control`
package contains a node to turn joystick commands into control
messages that drive the throttle and steering of the model. To use
this, launch a simulation as described above, then run the following:
```
roslaunch sd_control sd_twizy_control_teleop.launch
```

You can map specific buttons using the parameters defined in that
launch file. For instance, the following uses the left stick for
throttle, the right stick for steering, and right button (RB) to
enable control on a Logitech F710 Gamepad:
```
roslaunch sd_control sd_twizy_control_teleop.launch enable_button:=5 throttle_axis:=1 steer_axis:=2
```

### Keyboard
The simulation can also be controlled by the keyboard. To launch the sd_teleop_keyboard node, run the following:
```
./src/streetdrone_model/sd_control/keyboardlaunch.sh 
```
And follow the instructions on the terminal

### Display the robot only in rviz:
First, make sure you have rviz installed, by doing:
```
cd ~/catkin_ws
roscore
rviz
```
If you don't have rviz installed, do `sudo apt-get install ros-kinetic-rviz*`.  
The configuration file of the SD Twizy is located at `~/catkin_ws/src/streetdrone_model/sd_robot/config/sd_twizy.rviz`
