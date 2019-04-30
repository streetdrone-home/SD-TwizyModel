# sd_robot
This is the simulation package for the SD Twizy vehicle, containing different launch files of the model in various scenarios. It includes different gazebo world configurations and the launch file for the description of the model. 

### Sensors
LiDAR: VLP - 16 Velodyne  
Cameras: 8 x Sekonix SF332X Cameras  
The scripts for the sensors are written based on the common scripts that exist for sensors in Gazebo.

### Launch
The package includes three different world configurations built using the default Gazebo models.
Every launch file automatically opens both Gazebo and rViz, using the configuration: `config/sd_twizy_rviz.rviz`
It also includes the launch file for the model description including solely the URDF `roslaunch sd_robot sd_twizy_demo.launch`
To launch each world:

##### A. Default
`roslaunch sd_robot sd_twizy_default.launch`
<p align="center"> 
<img src="../sd_docs/imgs/default.png">
</p>

##### B. Shapes
`roslaunch sd_robot sd_twizy_shapes.launch`
<p align="center"> 
<img src="../sd_docs/imgs/small.png">
</p>

##### C. Large City
`roslaunch sd_robot sd_twizy_large_city.launch`
<p align="center"> 
<img src="../sd_docs/imgs/large.png">
</p>