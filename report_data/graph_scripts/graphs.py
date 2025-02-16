#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import matplotlib.pyplot as plt

# Initialize ROS node
rospy.init_node('compare_amcl_odom_paths')

# Load the bag file
bag = rosbag.Bag('ekf_bag2.bag', 'r')
bag2 = rosbag.Bag('ekf_bag.bag', 'r')

# Initialize lists to store data
odom_positions = []
amcl_positions = []

odom_path_x = []
odom_path_y = []

amcl_path_x = []
amcl_path_y = []

filtered_odom_positions = []
filtered_odom_path_x = []
filtered_odom_path_y = []

ground_truth_path_x = []
ground_truth_path_y = []


# Extract data from the bag
for topic, msg, t in bag2.read_messages(topics=['/odom']):
    pose = msg.pose.pose
    x, y = pose.position.x, pose.position.y
    odom_positions.append((x, y))
    odom_path_x.append(x)
    odom_path_y.append(y)

for topic, msg, t in bag2.read_messages(topics=['/amcl_pose']):
    pose = msg.pose.pose
    x, y = pose.position.x, pose.position.y
    amcl_positions.append((x, y))
    amcl_path_x.append(x)
    amcl_path_y.append(y)


for topic, msg, t in bag.read_messages(topics=['/odometry/filtered']):  # Replace with the actual topic for filtered odometry
    pose = msg.pose.pose
    x, y = pose.position.x, pose.position.y
    filtered_odom_positions.append((x, y))
    filtered_odom_path_x.append(x)
    filtered_odom_path_y.append(y)

for topic, msg, t in bag2.read_messages(topics=['/tf']):  # Replace with the actual Ground Truth topic
    if msg.transforms[0].header.frame_id == 'mocap':
       
        x = msg.transforms[0].transform.translation.x
        y = msg.transforms[0].transform.translation.y
        
        ground_truth_path_x.append(y - 1.25)
        ground_truth_path_y.append(-x + 0.85)





# Close the bag file
bag.close()
bag2.close()

# Plot the paths
plt.figure(figsize=(10, 5))
plt.plot(filtered_odom_path_x, filtered_odom_path_y, label='Filtered Odom Path', marker='x', markersize=3)
plt.plot(ground_truth_path_x, ground_truth_path_y, label='Ground Truth Path', marker='x', markersize=3)
plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.legend()
plt.title('Odometry vs Filtered Odom Path')
plt.grid()
plt.show()
