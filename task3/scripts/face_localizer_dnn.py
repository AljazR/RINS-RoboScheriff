#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from os.path import dirname, join
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

global face_poses

class face_localizer:
    def __init__(self):
        # Initialize the node
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Publiser for the visualization markers
        self.face_markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)

        # Publiser for the poster markers
        self.poster_markers_pub = rospy.Publisher('poster_markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Publisher for image of the detected face
        self.face_pub = rospy.Publisher('detected_face', Image, queue_size=1000)

    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected face

        k_f = 554 # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose

    def find_faces(self):
        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Tranform image to grayscale
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        #img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        #face_rectangles = self.face_detector(rgb_image, 0)
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.5:
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))

                # if face_distance > 1.2:
                #     continue

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

                # if the same pose is detected 3 times in the radius of 0.5m, then it is the same ring
                new_face = True
                # if you want to skip the check for the same face, set new_marker to True and comment out the for loop and if statement below
                new_marker = False
                for face_pose in face_poses:
                    if abs(face_pose[0] - pose.position.x) < 0.5 and abs(face_pose[1] - pose.position.y) < 0.5:
                        new_face = False
                        if face_poses[face_pose][1] < 3:
                            face_poses[face_pose][1] += 1
                            face_poses[face_pose][0].append(pose)
                            
                            if face_poses[face_pose][1] == 3:
                                new_marker = True
                                # Update the pose with the average of the last 3 poses
                                pose.position.x = sum([p.position.x for p in face_poses[face_pose][0]]) / 3
                                pose.position.y = sum([p.position.y for p in face_poses[face_pose][0]]) / 3
                                pose.position.z = sum([p.position.z for p in face_poses[face_pose][0]]) / 3
                        break 
                
                # If new ring is detected, add it to the dictionary
                if new_face:
                    face_poses[(pose.position.x, pose.position.y, pose.position.z)] = [[pose], 1]


                # Check if the face is already detected
                first_in_proximity = True
                for marker in self.marker_array.markers:
                    if abs(marker.pose.position.x - pose.position.x) < 0.5 and abs(marker.pose.position.y - pose.position.y) < 0.5:
                        first_in_proximity = False
                        print('Face already detected.')
                        break
           
                if pose is not None and new_marker and first_in_proximity:                    
                    
                    # TODO: check if it's a poster
                    
                    # Extract region containing face
                    face_region = rgb_image[y1:y2, x1:x2]

                    # Publish the image of the face
                    self.face_pub.publish(self.bridge.cv2_to_imgmsg(face_region, "bgr8"))
                    print(f'Pose of face [{pose.position.x}, {pose.position.y}] and distance {face_distance}')

                    # Create a marker used for visualization
                    self.marker_num += 1
                    marker = Marker()
                    marker.header.stamp = rospy.Time(0)
                    marker.header.frame_id = 'map'
                    marker.pose = pose
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.frame_locked = False
                    marker.lifetime = rospy.Time(0)
                    marker.id = self.marker_num
                    marker.scale = Vector3(0.1, 0.1, 0.1)
                    marker.color = ColorRGBA(0, 1, 0, 1)
                    self.marker_array.markers.append(marker)

                    self.face_markers_pub.publish(self.marker_array)
                elif pose is not None and new_marker and not first_in_proximity:
                    print("Face too close to another face.")


def main():
    global face_poses
    face_poses = {}    
    
    # Initialize face localizer node
    face_finder = face_localizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
