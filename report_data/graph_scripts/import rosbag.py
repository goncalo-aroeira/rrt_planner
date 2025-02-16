import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Path to your ROS bag file
bag_file = '/home/francisco/rrt_paths_2023-10-27-23-28-12.bag'

# Define the topics for actual and planned paths
actual_path_topic = '/amcl_pose'
planned_path_topic = '/move_base/DWAPlannerROS/local_plan'

# Initialize empty arrays to store the paths
actual_path = []
planned_path = []

# Load the ROS bag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == actual_path_topic:
            actual_path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        elif topic == planned_path_topic:
            planned_path.append([msg.poses[0].pose.position.x, msg.poses[0].pose.position.y])

# Convert path lists to NumPy arrays for easier processing
actual_path = np.array(actual_path)
planned_path = np.array(planned_path)

# Plot the actual and planned paths
plt.figure(figsize=(8, 6))
plt.plot(actual_path[:, 0], actual_path[:, 1], label='Actual Path', color='blue', linewidth=2)
plt.plot(planned_path[:, 0], planned_path[:, 1], label='Planned Path', color='red', linestyle='--', linewidth=2)

# Customize the plot as needed
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Actual vs. Planned Path')
plt.legend()

# Display the graph
plt.grid()
plt.show()
