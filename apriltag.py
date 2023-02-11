import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion

def main():

    def apriltagSubsciber(data):
        detectionArray = data.detections

        detection = detectionArray[0] #Assume one apriltag detection

        pose = detection.pose.pose.pose

        x = pose.position.x
        y = pose.position.y

        q = pose.orientation #as quaternion

        eulerAngles = euler_from_quaternion([q.x, q.y, q.z, q.w])

        yaw = eulerAngles[2]

        robotPosition = Vector3()

        robotPosition.x = x
        robotPosition.y = y
        robotPosition.z = yaw

        pose_pub.publish(robotPosition)

        



    rospy.Subscriber("/apriltag", AprilTagDetectionArray, apriltagSubsciber)

    pose_pub = rospy.Publisher("/pose", Vector3, queue_size=10)

    


main()