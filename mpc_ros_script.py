import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from mpc_ros import MPC
import numpy as np

path = []
state = []
map = []
res = 0
x_offset = 0
y_offset = 0
should_update_path = True

def path_subscriber(data):
    global path, should_update_path
    path = np.reshape(data.data, (len(data.data) // 2, 2))
    should_update_path = True

def position_subscriber(data):
    global state
    position = data.pose.pose.position
    rotation = data.pose.pose.orientation

    rotationEuler = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
    yaw = rotationEuler[2]

    state = [position.x, position.y, yaw]

def grid_subscriber(data):
    global map, res, x_offset, y_offset
    width = data.info.width
    height = data.info.height
    map = np.reshape(data.data, (height, width))
    res = data.info.resolution
    x_offset = data.info.origin.position.x
    y_offset = data.info.origin.position.y

def main():
    global path, state, map, res, x_offset, y_offset, should_update_path

    rospy.init_node('mpc_node')

    timesteps = 4
    sampleCount = 500
    bestSamples = 50
    iterations = 5

    frequency = 10 #hz

    mpc = None
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    rospy.Subscriber("/path", Float32MultiArray, path_subscriber)

    rospy.Subscriber("/odom", Odometry, position_subscriber)

    rospy.Subscriber("/map", OccupancyGrid, grid_subscriber)

    rate = rospy.Rate(frequency)

    velocity_twist = Twist() # A velocity to publish
    velocity_twist.linear.x = 0
    velocity_twist.linear.y = 0
    velocity_twist.linear.z = 0
    velocity_twist.angular.x = 0
    velocity_twist.angular.y = 0
    velocity_twist.angular.z = 0

    while not rospy.is_shutdown():
        if len(state) == 0 or len(map) == 0 or len(path) == 0:
            continue
        if mpc == None:
            print("initialize")
            mpc = MPC(timesteps, sampleCount, bestSamples, iterations, x_offset, y_offset, res, 1 / frequency)
        mpc.updateState(state)
        if should_update_path:
            mpc.updatePath(path)
            should_update_path = False
        mpc.updateGrid(map)
        linear_velocity, angular_velocity = mpc.control()
        velocity_twist.linear.x = linear_velocity
        velocity_twist.angular.z = angular_velocity
        velocity_publisher.publish(velocity_twist)
        print(linear_velocity, angular_velocity)
        rate.sleep()

if __name__ == "__main__":
    main()