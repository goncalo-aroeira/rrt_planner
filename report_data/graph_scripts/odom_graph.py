import rospy
import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

def extract_odometry_covariance_x(odometry_msg):
    # Extract the covariance value for X from the Odometry message
    covariance_x = odometry_msg.pose.covariance[0]  # Adjust the index as needed

    return covariance_x

def main():
    rospy.init_node('odometry_covariance_visualization', anonymous=True)
    bag_file = 'new_amcl_data_official.bag'
    topic_odometry = '/odom'  # Adjust to your topic for odometry data

    odometry_covariances_x = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic_odometry]):
            covariance_x = extract_odometry_covariance_x(msg)
            odometry_covariances_x.append(covariance_x)

    # Create a time vector for plotting
    time = [i for i in range(len(odometry_covariances_x))]

    # Plot odometry covariance values for X over time
    plt.figure()
    plt.plot(time, odometry_covariances_x, label='Odometry Covariance X')
    plt.xlabel('Time')
    plt.ylabel('Covariance Value')
    plt.title('Odometry Covariance (X) Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
