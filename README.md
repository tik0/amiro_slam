# AMiRo SLAM

Tested with Ubuntu 16.04 and ROS Kinetic.

* SLAM can be performed in three different setups
  * Simulation: `roslaunch amiro_slam start.launch`
  * In the TeleWorkBench: `roscp amiro_slam rsb.conf ~/rsb.conf && roslaunch amiro_slam start.launch use_sim_time:=false use_gazebo:=false`
  * From a bag file: `roslaunch amiro_slam start.launch use_gazebo:=false`

![AMiRo in TWB performing SLAM](/images/TWB_SLAM.jpg)

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

There are several parameter you have to adapt to your scenario in the 'start.launch'.

### Parameter start.launch

|         Name         | Default |                                                         Description                                                          |
| -------------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------- |
| use_sim_time         | true    | If this is set to true ros::time::now will be used as timestmap otherwise the timestmap has to come from gazebo or AMiRo.    |
| use_gazebo           | true    | To controll the AMiRo with an xbox controller                                                                                |
| marker_id            | 1       | TeleWorkBench marker ID attached to the AMiRo                                                                                |
| amiro_nr             | 1       | AMiRo number which also controls the topic prefix like `/amiro$(arg amiro_nr)/`                                              |
| xbox_controller      | 0       | To controll the AMiRo with an xbox controller                                                                                |
| keyboard_controller  | 0       | To controll the AMiRo with the keyboard                                                                                      |
| laserscaner          | 0       | If you want to use a laserscaner. With laserscan_sick and laserscan_hokuyo you can controll the which laserscan is used.     |
| laserscan_sick       | 0       | Decide to use the sick tim laserscaner.                                                                                      |
| laserscan_hokuyo     | 0       | Decide to use the hokuyo laserscanner.                                                                                       |
| twb                  | 0       | Use the TeleWorkBench data .                                                                                                 |
| robot_localization   | 0       | Use the robot_localization packge from ros which is a kalman-filter. It combines the AMiRo-odometry and the twb-data.        |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data                  |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                                      |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                                        |

As default there are only converter for one AMiRo. If you want to add more AMiRo you have to edit the 'start.launch' and copy a block of the 'slam.launch'.

### Parameter slam.launch

|         Name         | Default |                                                   Description                                                   |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------- |
| amiroNr              | 1       | Set the amiroId as namespace for all bridges.                                                                   |
| use_gazebo           | 0       | Disables the run of rsb_to_ros and ros_to_rsb bridges.                                                          |
| markerId             | 1       | Set the markerId from the twb data.                                                                             |
| laserscan            | 0       | Toggle the laserscaner.                                                                                         |
| laserscan_sick       | 0       | Toggle the sick tim laserscanner.                                                                               |
| laserscan_hokuyo     | 0       | Toggle the hokuyo laserscanner.                                                                                 |
| camera               | 0       | The the vision bridge for the inbuild AMiRo camera.                                                             |
| twb                  | 0       | Toggle the bridge for the twb data.                                                                             |
| robot_localization   | 0       | Toggle the extended kalman filter for AMiRo odometry and twb data.                                              |
| rostimeno            | 0       | Toggle the header stimestamps of each message with rostimenow or the normal rsb timestamps.                     |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data     |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                         |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                           |
| teleop               | 0       | Toggle the bridges to control the AMiRo via teleop.                                                             |
| nav_stack            | 0       | Toggle the ros navigation stack for the AMiRo.                                                                  |
| no_static_map        | 0       | If there is a dynamic map while SLAM'ing this has to be set 0 otherwise if there is a static map set this to 1. |
| hector_mapping       | 0       | Toggle SLAM'ing with the ros package hector_mapping.                                                            |
| octomap              | 0       | Toggle the ros package octomap with creates a map from the AMiRo odometry or the combined ekf data.             |
| gmapping             | 0       | Toggle SLAM'ing with the ros package gmapping.                                                                  |

## Nodes

### ros_laser_scan_to_ros_pointcloud2

This node contains a converter for ros::sensor_msgs::LaserScan to ros::sensor_msgs::PointCloud2.

### dynamic_tf_with_odom

This node contains dynamic transformer for ros.
It updates a transformation between two coordinate frames with a given ros::nav_msgs::Odometry.

#### Parameter

|          Name           |  Type  | Default |                                                            Description                                                            |
| ----------------------- | ------ | ------- | --------------------------------------------------------------------------------------------------------------------------------- |
| ros_listener_odom_topic | string | /topic  | Listenertopic for the ros::nav_msgs::Odometry to update the tf.                                                                   |
| parent_frame            | string | /parent | Parent frame id                                                                                                                   |
| child_frame             | string | /child  | Child frame id                                                                                                                    |
| rostimenow              | bool   | false   | If this is set to true, ros::now as headertimestamp will be used otherwhise the timestamp from the input ros::nav_msgs::Odometry. | 
