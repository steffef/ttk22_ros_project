#ifndef SLAM_H_
#define SLAM_H_

#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/iss_3d.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

class SLAM {
public:
    SLAM();

    /**
     * @brief 
     * 
     * @param pc 
     */
    void icp_alg(const pcl::PointCloud<pcl::PointXYZ>& pc);
    geometry_msgs::Pose get_pose_msg();

private:
    /**
     * @brief 
     * 
     * @param pc 
     * @param keypoints 
     */
    void extract_keypoints(const pcl::PointCloud<pcl::PointXYZ>& pc,
                            pcl::PointCloud<pcl::PointXYZ>& keypoints);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ> prev_keypoints;
    Eigen::Matrix4f transformationMatrix;
    geometry_msgs::Pose pose;
};

class Listener {
public:
    Listener();

    void callback(const sensor_msgs::PointCloud2ConstPtr& ptCloud);

private:
    SLAM slam;
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
};

#endif // SLAM_H_
