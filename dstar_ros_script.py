import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import dstar_ros
import numpy as np

def main():

    radius = 8 #robot rad (grid units)

    frequency = 10 #hz

    pose = np.array([])

    map = np.array([])

    update_map = False

    goal = np.array([])

    dstar = None

    path_publisher = rospy.Publisher("/path", Float32MultiArray, queue_size = 10)

    def grid_subscriber(data):
        global update_map, map
        width = data.info.width
        height = data.info.height
        map = np.reshape(data.data (height, width))
        update_map = True

    rospy.Subscriber("/map", OccupancyGrid, grid_subscriber)

    def position_subscriber(data):
        global pose
        position = data.pose.pose.position

        coords = [position.x, position.y]

        pose = coords

    rospy.Subscriber("/odom", Odometry, position_subscriber)

    rate = rospy.Rate(frequency)

    completedInitialRun = False

    path = []
    while not rospy.is_shutdown():
        if (not completedInitialRun and len(dstar.currentMap) > 0):
            dstar.find_path(True)
            completedInitialRun = True

        if (dstar.needs_new_path):
            path = dstar.createPathList()

        path_publisher.publish(path)

        rate.sleep()

if __name__ == "__main__":
    main()