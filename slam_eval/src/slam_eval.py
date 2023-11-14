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
    fig1, ax1 = plt.subplots(3)  # Create a figure with 3 subplots for position
    fig2, ax2 = plt.subplots(4)  # Create a separate figure with 4 subplots for orientation
    max_plot_length = 500

    while not rospy.is_shutdown():
        plot_length = min(len(slam_pose), len(gt_pose))
        if plot_length == 0:
            continue
        elif plot_length > max_plot_length:
            slam_pose = slam_pose[plot_length-max_plot_length:plot_length]
            gt_pose = gt_pose[plot_length-max_plot_length:plot_length]
            plot_length = max_plot_length

        for i in range(3):
            ax1[i].clear()
            slam_pose_i = slam_pose[:plot_length, i]
            gt_pose_i = gt_pose[:plot_length, i]
            ax1[i].plot(slam_pose_i, label='Estimated Pose', color='blue')
            ax1[i].plot(gt_pose_i, label='Actual Pose', color='red')
            ax1[i].legend()
            ax1[i].set_title('Estimated vs Actual Poses for coordinate {}'.format(i))

        for i in range(3, 7):
            ax2[i-3].clear()
            slam_pose_i = slam_pose[:plot_length, i]
            gt_pose_i = gt_pose[:plot_length, i]
            ax2[i-3].plot(slam_pose_i, label='Estimated Orientation', color='blue')
            ax2[i-3].plot(gt_pose_i, label='Actual Orientation', color='red')
            ax2[i-3].legend()
            ax2[i-3].set_title('Estimated vs Actual Orientation for coordinate {}'.format(i))

        plt.pause(0.1)  # Update plot every 0.1 seconds
if __name__ == '__main__':
    rospy.init_node('slam_eval')
    rospy.loginfo('slam_eval node started')

    rospy.Subscriber('/slam_pose_topic', Pose, slam_pose_callback)
    rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, gt_pose_callback)

    plot_eval()
    rospy.spin()