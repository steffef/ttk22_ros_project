#include <ros/ros.h>
#include <std_msgs/String.h>

#include "slam.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PcPcrocessor pcPcrocessor;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rmf_obelix/velodyne_points",10, &PcPcrocessor::callback, &pcPcrocessor);

  ros::spin();

  return 0;
}
