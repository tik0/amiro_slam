# AMiRo SLAM

Tested with Ubuntu 16.04 and ROS Kinetic.

![AMiRo in TWB performing SLAM](/images/TWB_SLAM.jpg)

* SLAM can be performed in three different setups
  * Simulation: `roslaunch amiro_slam start_gazebo.launch`
  * With the real AMiRo (with or without TeleWorkBench): `roscp amiro_slam rsb.conf ~/rsb.conf && roslaunch amiro_slam start_amiro.launch`
  * From a rsbag (not ROS bag) file: `roslaunch amiro_slam start_rsbag.launch`
* Steer the robot
  * [turtlebot_teleop_key](http://wiki.ros.org/turtlebot_teleop?distro=kinetic#Keyboard_Teleop) via `keyboard_controller` (standard): Use `u i o; j k l; m , .`
  * [turtlebot_teleop_joy](http://wiki.ros.org/turtlebot_teleop?distro=kinetic#PS3.2BAC8-XBox_Teleop) via `xbox_controller` : Use your Xbox controller
  * [move_base](http://wiki.ros.org/move_base) via `nav_stack`: Set 2dNavGoals in rviz
  * [frontier_exploration](http://wiki.ros.org/frontier_exploration) via `nav_stack` and `frontier_exploration`: Searches its own goal locations
* Hints
  * The robot's namespace is declared in the `start_*.launch` files. It controls the scopes/topics in simulation as well as in real life.
  * If you changed the namespace, the rviz configuration needs to be altered as well. E.g. if the current namespace is `amiro4` and you want to change all namespaces in rviz to `amiro9`, do: `$ sed 's#amiro4#amiro9#g' -i rviz/config.rviz`

## Necessary packages

* [amiro_robot](https://github.com/tik0/amiro_robot)
* [amiro_bridges](https://github.com/tik0/amiro_bridges) (only for real AMiRo setup)
* [hector_slam](http://wiki.ros.org/hector_slam)
* [frontier_exploration](http://wiki.ros.org/frontier_exploration)
* [octomap_mapping](http://wiki.ros.org/octomap_mapping)
* [slam_gmapping](http://wiki.ros.org/slam_gmapping)
* [turtlebot](http://wiki.ros.org/turtlebot)
* [move_base](http://wiki.ros.org/move_base)

## How To

There are several parameter you have to adapt to your scenario in the `configure.launch`.
The `start_*.launch` files configure `configure.launch` for various scenarios, so that it can be easily adapted.

## Nodes

### ros_laser_scan_to_ros_pointcloud2

This node contains a converter for ros::sensor_msgs::LaserScan to ros::sensor_msgs::PointCloud2.

### dynamic_tf_with_odom

This node contains dynamic transformer for ros.
It creates a transformation between two coordinate frames with a given ros::nav_msgs::Odometry.

#### Parameter

|          Name           |  Type  | Default |                                                            Description                                                            |
| ----------------------- | ------ | ------- | --------------------------------------------------------------------------------------------------------------------------------- |
| ros_listener_odom_topic | string | /topic  | Listenertopic for the ros::nav_msgs::Odometry to update the tf.                                                                   |
| parent_frame            | string | /parent | Parent frame id                                                                                                                   |
| child_frame             | string | /child  | Child frame id                                                                                                                    |
| rostimenow              | bool   | false   | If this is set to true, ros::now as headertimestamp will be used otherwhise the timestamp from the input ros::nav_msgs::Odometry. | 
