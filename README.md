# espeleo_collision
-------------------------

This repositry contains the packege for colision both in 2 or 3 dimenssions. The the data acquired that is used on the package comes from the laser installed on espeleo robo, the ouster vlp-16

 

## Scripts included on this package:
- 3dcollision.py : Considering the data in 3 dimenssions coming from the laser,the script calculate points of collision danger, and return massages of if is going to colide or not and the sector of collision.
- LaserScan.py : Converting the data from the laser to 2 dimenssions, it return the distance of points of collision.

## How to interact

First of all, if some modification are needed in the distances of collision, it is necessary to set some parameters at `colision_configuration.yaml`, located in `/colisao/Config`. 

Inside this file, there is two options on 2 and 3D. On the 2D model modifications of the collision range ( minimum and maximum distances) are possible.On the 3D model its possible change the distance used on  yellow and red LEDS distance (presents in the GUI), the pairt of points and the maximum slope angle to identify to identify a obstacle as an collision risk.
**Topics**

The collision in 2 dimenssions will publishi in the topic:
- `/colisao` : This topic shows the distance betewen the robot and the point of colision of the robot.

The collision in 3 dimenssions will publish in the topics:
- `/colisao` : It shows integer numbers of 0 to 2, where "0" represents a green LED, "1" yellow LED and "2" red LED.
- `/pontos` : Show numbers 0 to 6 where the numbers 1 to 6 shows the sector of the robot where are risk of collision : 1 is the front of the robot, 2 is left front coner, 3 the right front coner , 4 the rear left corner, the 5 rear right corner and 6 is the back part of the robot.
- `/Pontos_colisao` : Publish points classified as a collision risk from the `/point_cloud topic`. This topic can be used to see the points from the point cloud classified as collision risk on Rviz.

**Launch Files**

To run the part of the package responsable for collision in 2 dimenssions, first you need run the comand :
  -rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/os1_cloud_node/points
  the line above run the framework that converts the 3D point cloud to a 2D list of points.
After that you need to run the following launch:
- `Laser_colision.launch`: This launch run the code responsable to show the mensage of the points of collision in 2 dimenssionas. For run it correctly you need to modify the configuration file to 2D model.

To run the part of the packege responsable for collision in 3 dimenssions you need to run the launch:
- `detect_collision_3d.launch`: This launch is run the code for 3D collision points, and publish messages on the topics showed above, `/pontos`, `/colisao` and `/Pontos_colisao`. To run it correctly you must change de model in the configuration file for '3D'

