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

global ring_poses
global detection_rate

class The_Ring:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the depth topic
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback,  queue_size=1)
        
        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000, latch=True) # latch=True keeps the last message posted until a new message arrives

        # Publiser depth image with marked rings
        self.depth_rings_pub = rospy.Publisher('depth_rings', Image, queue_size=1)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Sound client
        # self.soundClient = SoundClient()

    def get_pose(self, e, dist, color):
        # if dist > 3:
        #     return

        global ring_poses
        # print(f"dist: {dist} color: {color}")

        # Calculate the position of the detected ellipse
        k_f = 554 # kinect focal length in pixels
        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]
        angle_to_target = np.arctan2(elipse_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

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
        # print(f"pose: {pose.position.x}, {pose.position.y}, {pose.position.z}")

        # if the same pose is detected 7 times in the radius of 0.5m, then it is the same ring
        new_ring = True
        new_marker = False
        for ring_pose in ring_poses:
            # if it's the same color, then the max distance can be bigger
            if color == ring_poses[ring_pose][2]:
                max_dist = 1.0
            else:
                max_dist = 0.5

            if abs(ring_pose[0] - pose.position.x) < max_dist and abs(ring_pose[1] - pose.position.y) < max_dist:
                new_ring = False
                if ring_poses[ring_pose][1] < 7:
                    ring_poses[ring_pose][1] += 1
                    ring_poses[ring_pose][0].append(pose)
                    
                    if ring_poses[ring_pose][1] == 7:
                        new_marker = True
                        # Update the pose with the average of the last 7 poses
                        pose.position.x = sum([p.position.x for p in ring_poses[ring_pose][0]]) / 7
                        pose.position.y = sum([p.position.y for p in ring_poses[ring_pose][0]]) / 7
                        pose.position.z = sum([p.position.z for p in ring_poses[ring_pose][0]]) / 7
                break        
        
        # If new ring is detected, add it to the dictionary
        if new_ring:
            ring_poses[(pose.position.x, pose.position.y, pose.position.z)] = [[pose], 1, color]

        # Check if the marker is already in the proximity of another marker
        first_in_proximity = True
        color_exists = False
        for marker in self.marker_array.markers:
            # if a marker with the same color already exists, throw away the new one
            if marker.color.r == 1 and marker.color.g == 0 and marker.color.b == 0 and color == "red" or \
                marker.color.r == 0 and marker.color.g == 1 and marker.color.b == 0 and color == "green" or \
                marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 1 and color == "blue" or \
                marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 0 and color == "black":
                color_exists = True
                break

            if abs(marker.pose.position.x - pose.position.x) < 0.5 and abs(marker.pose.position.y - pose.position.y) < 0.5:
                first_in_proximity = False
                

        if new_marker and first_in_proximity and not color_exists:
            # Create a marker
            self.marker_num += 1
            marker = Marker()
            marker.header.stamp = point_world.header.stamp
            marker.header.frame_id = point_world.header.frame_id
            marker.pose = pose
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Time(0)
            marker.id = self.marker_num
            marker.scale = Vector3(0.15, 0.15, 0.15)

            # Set the marker color
            if color == "red":
                marker.color = ColorRGBA(1, 0, 0, 1)
            elif color == "green":
                marker.color = ColorRGBA(0, 1, 0, 1)
            elif color == "blue":
                marker.color = ColorRGBA(0, 0, 1, 1)
            elif color == "black":
                marker.color = ColorRGBA(0, 0, 0, 1)
            self.marker_array.markers.append(marker)
            self.markers_pub.publish(self.marker_array)

            print(f"{color[0].upper() + color[1:]} ring detected.")
        elif new_marker and color_exists:
            print(f"{color[0].upper() + color[1:]} ring already detected.")
        elif new_marker and not first_in_proximity:
            print(f"{color[0].upper() + color[1:]} ring too close to another ring.")

    def depth_image_callback(self,data):
        global detection_rate
        if detection_rate > 0:
            detection_rate -= 1
            return
        else:
            detection_rate = 10
        
        # Convert the image message to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        try:
            cv_image = rospy.wait_for_message('/camera/rgb/image_raw', Image)
            cv_image = self.bridge.imgmsg_to_cv2(cv_image, "bgr8")
        except Exception as e:
            print(e)

        image_1 = depth_image / 65536.0 * 255
        image_1 = image_1/np.max(image_1) * 255
        depth_img = np.array(image_1, dtype= np.uint8)

        # Set the dimensions of the image
        self.dims = depth_img.shape

        # Binarize the image, there are different ways to do it
        thresh = cv2.adaptiveThreshold(depth_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
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
                if dist < 5:
                    candidates.append((e1,e2))

        # print("Found", len(candidates), "candidates for rings")

        depth_rgb = cv2.cvtColor(depth_img,cv2.COLOR_GRAY2RGB)

        # Extract the depth from the depth image
        for c in candidates:
            depth_rgb_copy = depth_rgb.copy()
            cv_image_copy = cv_image.copy()

            # The centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # Drawing the ellipses on the image
            cv2.ellipse(depth_rgb_copy, e2, (255, 0, 0), -1)
            cv2.ellipse(depth_rgb_copy, e1, (0, 255, 0), -1)

            cv2.ellipse(cv_image_copy, e2, (255, 0, 0), -1)
            cv2.ellipse(cv_image_copy, e1, (0, 255, 0), -1)

            # GET THE DEPTH OF THE RIM OF THE ELLIPSES
            # Get coordinates of the blue pixels in depth_img
            blue_pixels = np.where(depth_rgb_copy[:,:,0] == 255)
            # Get the depth of the blue pixels
            blue_depth = depth_image[blue_pixels[0], blue_pixels[1]]
            # Remove the 0 and 5 values
            blue_depth = blue_depth[blue_depth != 0]
            blue_depth = blue_depth[blue_depth != 5]
            # Find frequency of each values
            values, counts = np.unique(blue_depth, return_counts=True)
            # Find value with highest frequency
            rim_depth = np.mean(values)
            # print(f" RING-BLUE: {np.unique(blue_depth, return_counts=True)}")


            # GET THE DEPTH OF THE CENTER OF THE ELLIPSES
            # get coordinates of the green pixels in depth_img
            green_pixels = np.where(depth_rgb_copy[:,:,1] == 255)
            # Get the depth of the green pixels
            green_depth = depth_image[green_pixels[0], green_pixels[1]]
            # Find frequency of each value
            values, counts = np.unique(green_depth, return_counts=True)
            # Find out the rate of 0 and 5 -> if it is a ring, then the center has mostly 0 and 5 values
            count_0_5 = counts[0] + counts[-1]
            rate_0_5 = count_0_5/len(green_depth)
            # print(f" CENTER-GREEN: {np.unique(green_depth, return_counts=True)} rate: {rate_0_5}")


            if rate_0_5 > 0.8:
                # GET THE COLOR OF THE RING
                color_pixels = cv_image[blue_pixels[0], blue_pixels[1]]

                # Make mean over all pixels
                rim_color = np.mean(color_pixels, axis=0)
                # Get index of the max value
                max_index = np.argmax(rim_color)

                if np.std(rim_color) < 1:
                    color = "black"
                elif max_index == 0:
                    color = "blue"
                elif max_index == 1:
                    color = "green"
                elif max_index == 2:
                    color = "red"

                # print(f"{color} {rim_color}")
                self.get_pose(e1, rim_depth, color)

                # publish depth_img_copy
                self.depth_rings_pub.publish(self.bridge.cv2_to_imgmsg(depth_rgb_copy, "rgb8"))


        # if len(candidates) > 0:
        #     cv2.imshow("Ring image window", cv_image_copy)
        #     cv2.waitKey(1)


def main():
    global ring_poses
    ring_poses = {}
    
    global detection_rate
    detection_rate = 10

    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
