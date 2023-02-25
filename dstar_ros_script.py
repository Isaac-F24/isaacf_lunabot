import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import dstar_ros
import numpy as np
import matplotlib.pyplot as plt

pose = []

map = []

goal = []

update_map = False

res = 0

x_offset = 0

y_offset = 0

def grid_subscriber(data):
    global update_map, map, res, x_offset, y_offset
    width = data.info.width
    height = data.info.height
    map = np.reshape(data.data, (height, width))
    res = data.info.resolution
    x_offset = data.info.origin.position.x
    y_offset = data.info.origin.position.y
    update_map = True

def position_subscriber(data):
    global pose
    position = data.pose.pose.position

    coords = [position.x, position.y]
    pose = coords

def goal_subscriber(data):
    global goal
    print("GOAL")
    goal = [data.pose.position.x, data.pose.position.y]

def main():
    global map, update_map, pose, goal, res

    rospy.init_node('dstar_node')

    radius = 8 #robot rad (grid units)

    frequency = 10 #hz

    dstar = None

    path_publisher = rospy.Publisher("/path", Float32MultiArray, queue_size = 10)

    rospy.Subscriber("/map", OccupancyGrid, grid_subscriber)

    rospy.Subscriber("/odom", Odometry, position_subscriber)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_subscriber)

    rate = rospy.Rate(frequency)

    completedInitialRun = False

    path = []
    while not rospy.is_shutdown():
        if (dstar == None and len(map) > 0 and len(pose) > 0 and len(goal) > 0):
            dstar = dstar_ros.Dstar(goal, pose, map, radius, res, x_offset, y_offset)

        if (dstar != None):
            # print("Iterate")

            dstar.update_position(pose)

            if (completedInitialRun and update_map):
                dstar.update_map(map)
                update_map = False

            if (not completedInitialRun):
                dstar.find_path(True)
                completedInitialRun = True

            if (dstar.needs_new_path):
                path = np.array(dstar.createPathList())
                temp = map.copy()
                for point in path:
                    temp[point[0], point[1]] = -100
                # plt.imshow(temp, cmap='hot', interpolation='nearest')
                # plt.show()
                ros_struct = Float32MultiArray()
                ros_struct.data = path.flatten()
                path_publisher.publish(ros_struct)
                print("Publish Path")

        rate.sleep()

if __name__ == "__main__":
    main()