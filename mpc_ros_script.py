import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import mpc_ros

def main():

    timesteps = 4
    sampleCount = 500
    bestSamples = 50
    iterations = 5

    frequency = 10 #hz

    mpc = mpc_ros.MPC(timesteps, sampleCount, bestSamples, iterations, timePerTimestep= (1/frequency))

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    def path_subscriber(data):
        mpc.updatePath(data)

    rospy.Subscriber("/dstar_path", Float32MultiArray, path_subscriber)

    def position_subscriber(data):
        position = data.pose.pose.position
        rotation = data.pose.pose.orientation

        rotationEuler = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        yaw = rotationEuler[2]

        state = [position.x, position.y, yaw]

        mpc.updateState(state)

    rospy.Subscriber("/odom", Odometry, position_subscriber)

    def grid_subscriber(data):
        mpc.updateGrid(data)

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
        linear_velocity, angular_velocity = mpc.control()
        velocity_twist.linear.x = linear_velocity
        velocity_twist.angular.z = angular_velocity
        velocity_publisher.publish(velocity_twist)
        rate.sleep()

if __name__ == "__main__":
    main()