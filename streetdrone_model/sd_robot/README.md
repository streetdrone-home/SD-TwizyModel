# sd_robot
This is the simulation package for the SD Twizy vehicle, containing different launch files of the model in various scenarios. It includes different gazebo world configurations and the launch file for the description of the model. 

### Sensors
LiDAR: VLP - 16 Velodyne  
Cameras: 8 x Blackfly S 2.3MP
The scripts for the sensors are written based on the common scripts that exist for sensors in Gazebo.

### Launch
The package includes three different world configurations built using the default Gazebo models.
Every launch file automatically opens both Gazebo and rViz, using the configuration: `config/sd_twizy_rviz.rviz`
To launch each world:

##### A. Default
A default enviroment with a vehicle only
`roslaunch sd_robot sd_twizy_default.launch`
<p align="center"> 
<img src="../sd_docs/imgs/default.png">
</p>

##### B. Shapes
An environment with a selection of static objects and buildings
`roslaunch sd_robot sd_twizy_shapes.launch`
<p align="center"> 
<img src="../sd_docs/imgs/small.png">
</p>

##### c. Park
A Parkland Environment
`roslaunch sd_robot sd_twizy_park.launch`
<p align="center"> 
<img src="../sd_docs/imgs/park.png">
</p>

