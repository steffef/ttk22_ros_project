#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

slam_pose = np.empty((0, 7), float)
gt_pose = np.empty((0, 7), float)

def slam_pose_callback(msg):
    global slam_pose
    np_pose = np.array([[msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]])
    slam_pose = np.append(slam_pose, np_pose, axis=0)

def gt_pose_callback(msg):
    global gt_pose
    np_pose = np.array([[msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]])
    gt_pose = np.append(gt_pose, np_pose, axis=0)

def plot_eval():
    global slam_pose, gt_pose
    plt.ion()  
    fig1, ax1 = plt.subplots(3) 
    fig2, ax2 = plt.subplots(4)
    max_plot_length = 500
    str_array = ['x', 'y', 'z', 'w']

    while not rospy.is_shutdown():
        plot_length = min(len(slam_pose), len(gt_pose))
        if plot_length == 0:
            continue
        elif plot_length > max_plot_length:
            # Use the most updated poses
            slam_pose = slam_pose[len(slam_pose)-max_plot_length:len(slam_pose)]
            gt_pose = gt_pose[len(gt_pose)-max_plot_length:len(gt_pose)]
            plot_length = max_plot_length

        for i in range(3):
            ax1[i].clear()
            slam_pose_i = slam_pose[:plot_length, i]
            gt_pose_i = gt_pose[:plot_length, i]
            ax1[i].plot(gt_pose_i, label='Ground Truth Pos', color='red')
            ax1[i].plot(slam_pose_i, label='Slam Estimated Pos', color='blue')
            ax1[i].legend()
            ax1[i].set_title('Estimated vs Actual Position for coordinate ' + str_array[i])

        for i in range(3, 7):
            ax2[i-3].clear()
            slam_pose_i = slam_pose[:plot_length, i]
            gt_pose_i = gt_pose[:plot_length, i]
            ax2[i-3].plot(gt_pose_i, label='Ground Truth Orientation', color='red')
            ax2[i-3].plot(slam_pose_i, label='Slam Estimated Orientation', color='blue')
            ax2[i-3].legend()
            ax2[i-3].set_ylim([-1.1, 1.1]) 
            ax2[i-3].set_title('Estimated vs Actual Orientation for coordinate ' + str_array[i-3])

        plt.pause(0.1)


if __name__ == '__main__':
    rospy.init_node('slam_eval')
    
    rospy.Subscriber('/slam_pose_topic', Pose, slam_pose_callback)
    rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, gt_pose_callback)

    plot_eval()
    rospy.spin()