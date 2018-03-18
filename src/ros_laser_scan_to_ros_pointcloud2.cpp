// ============================================================================
// Name        : ros_laser_scan_to_ros_pointcloud2.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : This tool directly converts data provided as laser_scan into pointcloud2.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

string laserscan_topic;
string pointcloud2_topic;
string target_frame;
ros::Publisher pointcloud2;
laser_geometry::LaserProjection projector;


const string programName = "ros_laser_scan_to_ros_pointcloud2";

void convert(const sensor_msgs::LaserScan::ConstPtr& scan){
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(*scan, cloud);
  pointcloud2.publish(cloud);
}

int main(int argc, char * argv[]){
  ROS_INFO("Start: %s", programName.c_str());

  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("laserscan_topic",laserscan_topic,"/amiro/laserscan");
  node.param<string>("pointcloud2_topic",pointcloud2_topic,"/amiro/pointcloud2");
  ROS_INFO("laserscan_topic: %s",laserscan_topic.c_str());
  ROS_INFO("pointcloud2_topic: %s",pointcloud2_topic.c_str());

  pointcloud2 = node.advertise<sensor_msgs::PointCloud2>(pointcloud2_topic, 1);
  ros::Subscriber laser = node.subscribe(laserscan_topic, 1, convert);

  ros::spin();
  return 0;
}
