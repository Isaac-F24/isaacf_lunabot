import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import dstar_ros
import numpy

def main():

    goal = [0, 0]
    start = [0, 0]
    init_map = numpy.array([])

    radius = 8 #robot rad (grid units)

    frequency = 10 #hz

    dstar = dstar_ros.Dstar(goal, start, init_map, radius)

    path_publisher = rospy.Publisher("/path", Float32MultiArray, queue_size = 10)

    def grid_subscriber(data):
        dstar.update_map(data)

    rospy.Subscriber("/map", OccupancyGrid, grid_subscriber)

    def position_subscriber(data):
        position = data.pose.pose.position

        coords = [position.x, position.y]

        dstar.update_position(coords)

    rospy.Subscriber("/odom", Odometry, position_subscriber)

    rate = rospy.Rate(frequency)

    completedInitialRun = False

    path = []
    while not rospy.is_shutdown():
        if (not completedInitialRun and len(init_map) > 0):
            dstar.find_path(True)
            completedInitialRun = True

        if (dstar.needs_new_path):
            path = dstar.createPathList()

        path_publisher.publish(path)

        rate.sleep()

if __name__ == "__main__":
    main()