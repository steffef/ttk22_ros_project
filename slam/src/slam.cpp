#include "slam.h"

SLAM::SLAM() : icp() {
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setEuclideanFitnessEpsilon(1.0);
    icp.setRANSACOutlierRejectionThreshold(1.5);
}

void SLAM::icp_alg(const pcl::PointCloud<pcl::PointXYZ>& pc) {
    if (prev_keypoints.empty()) {
        std::cout << "Initializing keypoints" << std::endl;
        //extract_keypoints(pc, prev_keypoints);
        prev_keypoints = pc;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointCloud<pcl::PointXYZ> keypoints;
    //extract_keypoints(pc, keypoints);
    keypoints = pc;

    icp.setInputSource(prev_keypoints.makeShared());
    icp.setInputTarget(keypoints.makeShared());
    icp.align(transformed_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged. The fitness score is " << icp.getFitnessScore() << std::endl;

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
            std::cout << "Initializing pose" << std::endl;
            tf::poseTFToMsg(tf_transform, pose);
        } else {
            tf::Transform tf_pose;
            tf::poseMsgToTF(pose, tf_pose);
            tf_pose = tf_transform * tf_pose;
            tf::poseTFToMsg(tf_pose, pose);
        }

        prev_keypoints = keypoints; // Update the previous point cloud
    } else {
        std::cout << "ICP did not converge. Fitness score: " << icp.getFitnessScore() << std::endl;
    }
}

 void SLAM::extract_keypoints(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                pcl::PointCloud<pcl::PointXYZ>& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_keypoint;
    iss_keypoint.setInputCloud(pc.makeShared());
    iss_keypoint.setNonMaxRadius(0.2);
    iss_keypoint.setSalientRadius(0.5);
    iss_keypoint.setThreshold21(0.975);
    iss_keypoint.setThreshold32(0.975);
    iss_keypoint.setMinNeighbors(5);

    iss_keypoint.compute(keypoints);
}

geometry_msgs::Pose SLAM::get_pose_msg() {
    return pose;
}

Listener::Listener() : nh(), pose_pub(nh.advertise<geometry_msgs::Pose>("slam_pose_topic", 1)) {
}

void Listener::callback(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*ptCloud, cloud);

    slam.icp_alg(cloud);
    geometry_msgs::Pose slam_pose = slam.get_pose_msg();

    pose_pub.publish(slam_pose);
}
