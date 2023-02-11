import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import numpy as np
from mpc import MPC

def main():
    timesteps = 0
    best_samples = 0
    samples = 0
    iterations = 0
    dt = 0


    mpc = MPC(timesteps, best_samples, samples, iterations, dt)
    path = np.array([])
    pose = []

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    def path_sub(data):
        data = data.data
        path = np.array(data).reshape((len(data//2),2))
    
    def pose_sub(data):
        pose = np.array([data.x, data.y, data.z])

    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/path", Float32MultiArray, path_sub)
    rospy.Subscriber("/pose", Vector3, pose_sub)
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mpc.update_pose(pose)
        mpc.update_path(path)
        lin_vel, ang_vel = mpc.control()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel
        vel_pub.publish(twist)
        rate.sleep()

if __name__ == "__main__":
    main()