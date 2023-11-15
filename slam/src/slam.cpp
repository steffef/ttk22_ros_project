#include "slam.h"

SLAM::SLAM() : icp() {
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setEuclideanFitnessEpsilon(1.0);
    icp.setRANSACOutlierRejectionThreshold(0.01);
}

void SLAM::icp_alg(const pcl::PointCloud<pcl::PointXYZ>& pc) {
    if (prev_keypoints.empty()) {
        extract_keypoints(pc, prev_keypoints);
        //prev_keypoints = pc;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointCloud<pcl::PointXYZ> keypoints;
    extract_keypoints(pc, keypoints);
    //keypoints = pc;

    icp.setInputCloud(prev_keypoints.makeShared());
    icp.setInputTarget(keypoints.makeShared());
    icp.align(transformed_cloud);

    if (icp.hasConverged()) {
        transformationMatrix = icp.getFinalTransformation();
        Eigen::Quaternionf quat(transformationMatrix.block<3, 3>(0, 0));

        tf::Transform tf_transform;
        tf_transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
        tf_transform.setOrigin(tf::Vector3(transformationMatrix(0, 3),
                                           transformationMatrix(1, 3),
                                           transformationMatrix(2, 3)));

        // Check if pose orientation is valid
        if (pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0 &&
            pose.orientation.w == 0) {

            // Set initial pose
            tf::poseTFToMsg(tf_transform, pose);
        } else {
            tf::Transform tf_pose;
            tf::poseMsgToTF(pose, tf_pose);
            tf_pose = tf_transform * tf_pose;
            tf::poseTFToMsg(tf_pose, pose);
        }

        prev_keypoints = keypoints; // Update the previous point cloud
    } else {
        ROS_WARN("ICP did not converge.");
    }
}

 void SLAM::extract_keypoints(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                pcl::PointCloud<pcl::PointXYZ>& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_keypoint;

    if(pc.empty()) {
        ROS_WARN("Point cloud is empty.");
        return;
    }

    double resolution = 0.05; // Treated as tuning parameter
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    iss_keypoint.setSearchMethod(tree);
    iss_keypoint.setSalientRadius(6 * resolution);
    iss_keypoint.setNonMaxRadius(4 * resolution);
    iss_keypoint.setThreshold21(0.975);
    iss_keypoint.setThreshold32(0.975);
    iss_keypoint.setMinNeighbors(5);
    iss_keypoint.setInputCloud(pc.makeShared());
    iss_keypoint.compute(keypoints);
}

geometry_msgs::Pose SLAM::get_pose_msg() {
    return pose;
}

PcPcrocessor::PcPcrocessor() : nh(), pose_pub(nh.advertise<geometry_msgs::Pose>("slam_pose_topic", 1)) {
}

void PcPcrocessor::callback(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
    
    // Transform from velodyne to navigation frame
    sensor_msgs::PointCloud2 transformed_cloud;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped tf_stamped;
    try {
        tf_stamped = tfBuffer.lookupTransform("navigation", "rmf_obelix/rmf_obelix/velodyne", ros::Time(0));
        tf2::doTransform(*ptCloud, transformed_cloud, tf_stamped);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep(); // Ensure that transform gets published
    }

    // Convert from PointCloud2 to PointCloud to use with PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);

    slam.icp_alg(cloud);
    geometry_msgs::Pose slam_pose = slam.get_pose_msg();

    pose_pub.publish(slam_pose);
}