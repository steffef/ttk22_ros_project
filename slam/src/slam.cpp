#include "slam.h"

SLAM::SLAM() : icp() {
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(20);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(1.0);
    icp.setRANSACOutlierRejectionThreshold(0.01);
}

void SLAM::icp_alg(const pcl::PointCloud<pcl::PointXYZ>& pc) {
    if (pc.empty()) {
        ROS_WARN("Point cloud is empty.");
        return;
    }
    else if (prev_keypoints.empty()) {
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

        // Check if pose orientation is valid
        if (pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0 &&
            pose.orientation.w == 0) {

            geometry_msgs::Pose init_pose;
            init_pose.orientation = tf2::toMsg(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
            init_pose.position.x = transformationMatrix(0, 3);
            init_pose.position.y = transformationMatrix(1, 3);
            init_pose.position.z = transformationMatrix(2, 3);
            
            pose = init_pose; // Set initial pose
        } else {
            geometry_msgs::Pose transformed_pose;
            geometry_msgs::TransformStamped tf_transform_stamped;
            tf_transform_stamped.transform.rotation = tf2::toMsg(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
            tf_transform_stamped.transform.translation.x = transformationMatrix(0, 3);
            tf_transform_stamped.transform.translation.y = transformationMatrix(1, 3);
            tf_transform_stamped.transform.translation.z = transformationMatrix(2, 3);
            tf2::doTransform(pose, transformed_pose, tf_transform_stamped);
            pose = transformed_pose; // Update pose with new transformation
        }

        prev_keypoints = keypoints; // Update the previous point cloud
    } else {
        ROS_WARN("ICP did not converge.");
    }
}

 void SLAM::extract_keypoints(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                pcl::PointCloud<pcl::PointXYZ>& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

    double pc_resolution = 0.05; // Treated as tuning parameter
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(6 * pc_resolution);
    iss_detector.setNonMaxRadius(4 * pc_resolution);
    iss_detector.setThreshold21(0.975);
    iss_detector.setThreshold32(0.975);
    iss_detector.setMinNeighbors(5);
    iss_detector.setNumberOfThreads(4);
    iss_detector.setInputCloud(pc.makeShared());
    iss_detector.compute(keypoints);
}

geometry_msgs::Pose SLAM::get_pose_msg() {
    return pose;
}

PcPcrocessor::PcPcrocessor() : nh(), pose_pub(nh.advertise<geometry_msgs::Pose>("slam_pose", 1)) {
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

    // Make sure to only publish valid poses
    if (slam_pose.orientation.x == 0 && slam_pose.orientation.y == 0 && slam_pose.orientation.z == 0 &&
        slam_pose.orientation.w == 0) {} // Do nothing
    else{
        pose_pub.publish(slam_pose);
    }
}