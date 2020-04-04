# SD Twizy  Vehicle Simulation

Gazebo simulation packages for the SD Twizy vehicle

## Requirements

- Ubuntu 18.04 LTS
- ROS Melodic [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu)

This model, specifically the Gazebo plugin that implements the control
interface, requires Gazebo 9, which is the default version for ROS
Melodic, and will be installed with `rosdep` below.

## Building

* Go to the `src` directory of your workspace and clone this
  repository or copy its contents.
* When using Git, check out this branch:

        cd SD-TwizyModel && git checkout melodic

* Make sure all required dependecies are installed using
  [`rosdep`](http://wiki.ros.org/rosdep). From the top directory of
  your catkin workspace run (assuming this repository is cloned
  directly under `src`):

        rosdep update && rosdep install --from-paths src/SD-TwizyModel/ --ignore-src -r -y

  Now you can build the packages in this repository. `catkin_make`,
  [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/installing.html)
  and
  [`colcon`](https://catkin-tools.readthedocs.io/en/latest/installing.html)
  should all work.

### Autoware

The `sd_autoware` package included in this repository provides an
interface to be able to control the StreetDrone model using the
[Autoware.AI](https://catkin-tools.readthedocs.io/en/latest/installing.html)
open-source self-driving car stack.

By default this package is not built to prevent any problems when
Autoware.AI is not available. If it is available and you do want to
use the package, remove the file
`streetdrone_model/sd_autoware/CATKIN_IGNORE` and build again.

## Launch the simulation
The following launches the vehicle model in Gazebo and RViz for
visualizing the sensors' output:

```
roslaunch sd_bringup sd_twizy_gazebo.launch
```

<p align="center"> <img src="streetdrone_model/sd_docs/imgs/sd.png">
</p>

There are several more launch files included to provide a more
interesting surrounding for the vehicle (see known issues below to
make sure they work); try for instance:

```
roslaunch sd_bringup sd_twizy_gazebo_default.launch
```

or:

```
roslaunch sd_bringup sd_twizy_large_city.launch
```

### Display the robot only in Gazebo

You can use the `sd_twizy_spawn.launch` to only launch the model in an
already running instance of Gazebo. For instance, open an empty world
model:

    roslaunch gazebo_ros empty_world.launch

and then launch the model with:

    roslaunch sd_bringup sd_twizy_spawn.launch

#### Known Issues:

* `[Err] [REST.cc:205] Error in REST request` - this is a known error
    in Gazebo 9. To fix it, edit `~/.ignition/fuel/config.yaml` and
    change the server URL in there from https://api.ignitionfuel.org
    to https://api.ignitionrobotics.org
* `Couldn't open joystick force feedback!` - this error is non-fatal
    and can be ignored. It's a common error in the `joystick_drivers`
    node

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

### Autoware.AI
If the `sd_autoware` package is built, as described above, run the
following to launch the required nodes to be able to work with
Autoware.AI:
```
roslaunch sd_autoware sd_autoware.launch
```

This launches a node that translates Autoware.AI's `ctrl_cmd` topic to
`sd_control` messages, with a PID controller in between to regulate
the throttle based on the desired linear velocity. This means that the
model can be directly controlled using the output of Autoware.AI's
Pure Pursuit or MPC nodes.
