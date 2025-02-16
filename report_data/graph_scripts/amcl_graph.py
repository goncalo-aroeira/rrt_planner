import rospy
import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

def extract_covariance(estimated_pose):
    # Extract the covariance values from the PoseWithCovarianceStamped message
    covariance_matrix = estimated_pose.pose.covariance
    # Covariance values are stored in a 1D array (row-major order), convert it to a 2D matrix
    covariance_matrix = np.array(covariance_matrix).reshape(6, 6)
    return covariance_matrix

def main():
    rospy.init_node('covariance_visualization', anonymous=True)
    bag_file = 'new_amcl_data_official.bag'
    topic_estimated_pose = '/amcl_pose'  # Adjust to your topic

    covariances = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic_estimated_pose]):
            estimated_covariance = extract_covariance(msg)
            covariances.append(estimated_covariance)

    # Create a time vector for plotting
    time = [i for i in range(len(covariances))]

    # Extract individual covariance values for plotting
    cov_x = [cov[0, 0] for cov in covariances]
    cov_y = [cov[1, 1] for cov in covariances]
    cov_theta = [cov[5, 5] for cov in covariances]

    # Plot covariance values over time
    plt.figure()
    plt.plot(time, cov_x, label='Covariance X')
    plt.plot(time, cov_y, label='Covariance Y')
    plt.plot(time, cov_theta, label='Covariance Theta')
    plt.xlabel('Time')
    plt.ylabel('Covariance Value')
    plt.title('Covariance Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
