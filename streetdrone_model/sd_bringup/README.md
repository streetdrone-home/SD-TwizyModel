# sd_bringup
This package contains configuration files to run the SD Twizy vehicle
in various scenarios. It includes different gazebo world
configurations and launch files for the description of the model.

## Launch
The package contains several launch files to run different example of
the model in different worls, or just launch on it own. The launch
files with `*_gazebo_*` in their name automatically open both Gazebo
and RViz, using the configuration: `config/sd_twizy_rviz.rviz`. It
also includes a launch file that soleley spawns the URDF model
description, expecting Gazebo to already be running: `roslaunch
sd_bringup sd_twizy_spawn.launch`

To launch each example world:

##### A. Default
`roslaunch sd_bringup sd_twizy_gazebo_default.launch`
<p align="center"> 
<img src="../sd_docs/imgs/default.png">
</p>

##### B. Shapes
`roslaunch sd_bringup sd_twizy_gazebo_shapes.launch`
<p align="center"> 
<img src="../sd_docs/imgs/small.png">
</p>

##### C. Large City
`roslaunch sd_bringup sd_twizy_gazebo_large_city.launch`
<p align="center"> 
<img src="../sd_docs/imgs/large.png">
</p>
