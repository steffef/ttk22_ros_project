#ifndef SLAM_H_
#define SLAM_H_

#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/iss_3d.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

class SLAM {
public:
    SLAM();

    /**
     * @brief runs icp and updates private variables in
     * the SLAM class. This includes the pose
     * 
     * @param pc point cloud
     */
    void icp_alg(const pcl::PointCloud<pcl::PointXYZ>& pc);
    geometry_msgs::Pose get_pose_msg();

private:
    /**
     * @brief ISS keypoint detection
     * 
     * @param pc point cloud 
     * @param keypoints ret value after keypoint detection
     */
    void extract_keypoints(const pcl::PointCloud<pcl::PointXYZ>& pc,
                            pcl::PointCloud<pcl::PointXYZ>& keypoints);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ> prev_keypoints;
    Eigen::Matrix4f transformationMatrix;
    geometry_msgs::Pose pose;
};

class PcPcrocessor {
public:
    PcPcrocessor();

    /**
     * @brief callback function for the point cloud subscriber. It
     * also calls the slam algorithm and publishes the pose
     * 
     * @param ptCloud point cloud
     */
    void callback(const sensor_msgs::PointCloud2ConstPtr& ptCloud);

private:
    SLAM slam;
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    tf2_ros::Buffer tfBuffer;
};

#endif // SLAM_H_
