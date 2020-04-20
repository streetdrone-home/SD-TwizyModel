# sd_robot
This is the simulation package for the SD Twizy vehicle, containing different launch files of the model in various scenarios. It includes different gazebo world configurations and the launch file for the description of the model. 

### Sensors
LiDAR: VLP - 16 Velodyne  
Cameras: 8 x Blackfly S 2.3MP  
The scripts for the sensors are written based on the common scripts that exist for sensors in Gazebo.

### Launch
Three different launching configurations of the vehicle model in Gazebo.

##### Empty world: Launching Gazebo and RViz
`roslaunch sd_robot sd_twizy_empty.launch`  
Spawns the model in Gazebo and visualizes the sensors output in RViz, using the default config: `config/sd_twizy_rviz.rviz`

##### Empty world: Launching only Gazebo
`roslaunch sd_robot sd_twizy_empty_gazebo.launch`  
Only launches the model on an empty world in Gazebo.

##### Shapes
`roslaunch sd_robot sd_twizy_shapes.launch`  
A simple world built using default Gazebo models (shapes, houses etc).
<p align="center"> 
<img src="../sd_docs/imgs/small.png">
</p>
