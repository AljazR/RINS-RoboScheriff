#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String


class The_Ring:
    def __init__(self):
        rospy.init_node('detect_rings_arm', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/arm_camera/rgb/image_raw", Image, self.image_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('parking_markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Publisher for pose
        self.pose_pub = rospy.Publisher('arm_ring_pose', Pose, queue_size=1000, latch=True)

        # Arm position subscriber
        self.arm_position_sub = rospy.Subscriber("/arm_position", String, self.new_arm_position)
        self.arm_extended = False

    def new_arm_position(self, data):
        if data.data == 'extend':
            self.arm_extended = True
        else:
            self.arm_extended = False
        print(self.arm_extended)

    def get_pose(self,e,dist):
        # Calculate the position of the detected ellipse
        k_f = 525 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        first_in_proximity = True
        for marker in self.marker_array.markers:
            if abs(marker.pose.position.x - pose.position.x) < 0.5 and abs(marker.pose.position.z - pose.position.z) < 0.5:
                first_in_proximity = False
                break   

        if first_in_proximity:
            # Create a marker used for visualization
            self.marker_num += 1
            marker = Marker()
            marker.header.stamp = point_world.header.stamp
            marker.header.frame_id = point_world.header.frame_id
            marker.pose = pose
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Time(0)
            marker.id = self.marker_num
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(0, 1, 0, 1)
            self.marker_array.markers.append(marker)

            self.markers_pub.publish(self.marker_array)

            self.pose_pub.publish(pose)

    def image_callback(self,data):
        if not self.arm_extended:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                #             print dist
                if dist < 5:
                    candidates.append((e1,e2))

        try:
            depth_img = rospy.wait_for_message('/arm_camera/depth/image_raw', Image)
        except Exception as e:
            print(e)

        # Extract the depth from the depth image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")

            self.get_pose(e1, float(np.mean(depth_image[x_min:x_max,y_min:y_max])))

        if len(candidates)>0:
                cv2.imshow("Image window",cv_image)
                cv2.waitKey(1)


def main():
    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
