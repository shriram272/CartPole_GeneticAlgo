import rospy
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkStates

# Global variables to store cart and pole information
cart_pose = Pose()
pole_twist = Twist()
cart_pose_x = 0
y_angular = 0
pole_tip_pose_z = 0

# ROS Publisher for cart control
pub_cart = rospy.Publisher('/cart_controller/command', Float64, queue_size=10)

# Callback function to get cart pose from Gazebo
def get_cart_pose(data):
    global cart_pose, pole_twist, cart_pose_x, y_angular, pole_tip_pose_z
    ind = data.name.index('cart_pole::cart_link')
    cart_pose = data.pose[ind]

    ind_pitch = data.name.index('cart_pole::pole_link')
    pole_twist = data.twist[ind_pitch]

    ind_tip = data.name.index('cart_pole::tip_link')
    pole_tip_pose = data.pose[ind_tip]

    cart_pose_x = cart_pose.position.x
    y_angular = pole_twist.angular.y
    pole_tip_pose_z = pole_tip_pose.position.z

# Function to simulate the system with given PID values and plot the impulse response
def simulate_system(Kp_y, Ki_y, Kd_y, Kp_p, Ki_p, Kd_p):
    cart_position_history = []
    pole_angle_history = []
    time_points = []
    time_interval = 0.005

    # Reset simulation
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation_client()

    # Apply an impulse disturbance
    rospy.sleep(1.0)  # Wait for the system to stabilize
    pub_cart.publish(Float64(100.0))  # Apply a force to the cart

    # Simulate the system
    for i in range(1000):
        # Simulate the system with given PID values
        # Update cart_pose_x, y_angular, pole_tip_pose_z accordingly
        time_points.append(i * time_interval)
        cart_position_history.append(cart_pose_x)
        pole_angle_history.append(y_angular)

    # Plot the impulse response
    plt.figure()
    plt.plot(time_points, cart_position_history, label='Cart Position')
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('System Response to Impulse Disturbance')
    plt.grid(True)
    plt.show()

    return time_points, cart_position_history, pole_angle_history

# Main function
if __name__ == '__main__':
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)

    # Given PID values
    Kp_y = 16.49941092470197
    Ki_y = 16.36732391035991
    Kd_y = 11.781030785993568
    Kp_p = 0.5807028451044702
    Ki_p = 8.631333636926616
    Kd_p = 10.830558643289015

    # Simulate the system and plot the impulse response
    simulate_system(Kp_y, Ki_y, Kd_y, Kp_p, Ki_p, Kd_p)
