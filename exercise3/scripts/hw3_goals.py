#!/usr/bin/python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped
from sound_play.libsoundplay import SoundClient

new_face = False
found_faces = 0

def calibrate():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0.4
    goal.target_pose.pose.orientation.w = 1.0
    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Robot calibrated.")

def approach_face(marker_array: MarkerArray):
    global new_face
    global found_faces
    new_face = True

    #location of the face
    face_position = marker_array.markers[-1].pose.position
    face_orientation = marker_array.markers[-1].pose.orientation

    # location of the robot
    robot_location = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    robot_position = robot_location.pose.pose.position
    robot_orientation = robot_location.pose.pose.orientation

    # find cloest point facing the face
    new_x = [face_position.x + 0.5, face_position.x - 0.5, face_position.x, face_position.x]
    new_y = [face_position.y, face_position.y, face_position.y + 0.5, face_position.y - 0.5]
    new_w = [0, 1, 0.7, 0.7]
    new_z = [1, 0, -0.7, 0.7]
    # angles = [0, math.pi / 2, math.pi, 3 * math.pi / 2]
    min_dist = 1000
    # angle = 0
    for i in range(4):
        dist = math.sqrt((new_x[i] - robot_position.x)**2 + (new_y[i] - robot_position.y)**2)
        if dist < min_dist:
            min_dist = dist
            x = new_x[i]
            y = new_y[i]
            w = new_w[i]
            z = new_z[i]
            # angle = angles[i]

    # calclate rotation
    # q = q = tf.transformations.quaternion_about_axis(angle, axis)
    # axis = [1.0, 0.0, 0.0]
    # q = [math.cos(angle/2), math.sin(angle/2)*axis[0], math.sin(angle/2)*axis[1], math.sin(angle/2)*axis[2]]

    rospy.loginfo(f"Approaching point [{x:.3f}, {y:.3f}].")

    # send the robot to the face
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Approached face and saying hi.")
        sound_client = SoundClient()
        rospy.sleep(1)
        sound_client.say("Hello! You are a newly discovered face!")
        rospy.sleep(4)
    else:
        rospy.loginfo(f"Failed to approach face.")

    found_faces += 1
    new_face = False


def move_to_goal():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    rospy.Subscriber('face_markers', MarkerArray, approach_face)

    # Goal coordinates
    x_position =    [-0.64, -1.27, -0.07,  0.15,  0.14,  0.89,  1.92,  2.87,  2.00,  1.11,  1.57,  2.58,  2.58,  1.82,  0.95, -0.03, -0.83, -1.54, -1.45]
    y_position =    [ 0.95,  0.05, -0.33, -0.78, -1.33, -1.39, -1.11, -0.81,  0.67,  1.45,  0.35,  1.10,  2.06,  2.51,  2.55,  2.67,  1.69,  2.70,  1.68]
    z_orientation = [-0.98,  0.86, -0.02, -0.99, -0.46,  0.54, -0.58, -0.66,  0.77,  0.15, -0.99,  0.93,  0.16, -0.73,  0.70, -0.60, -0.24,  0.07, -0.76]
    w_orientation = [ 0.21,  0.51,  0.99,  0.08,  0.89,  0.84,  0.81,  0.75,  0.63,  0.99,  0.13,  0.37,  0.99,  0.68,  0.71,  0.80,  0.99,  0.99,  0.65]

    finished_goals = 0
    i = 0
    while i < len(x_position) and found_faces < 3:
        # Create a goal message
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x_position[i]
        goal.target_pose.pose.position.y = y_position[i]
        goal.target_pose.pose.orientation.z = z_orientation[i]
        goal.target_pose.pose.orientation.w = w_orientation[i]

        # Send the goal and wait for the result
        client.send_goal(goal)

        # Print the status of the goal
        print_log = True
        while not rospy.is_shutdown() and not new_face:
            status = client.get_state()
            if status == GoalStatus.ACTIVE and print_log:
                rospy.loginfo(f"Goal {i} is being processed.")
                print_log = False
            elif status == GoalStatus.PENDING:
                rospy.loginfo(f"Goal {i} is pending.")
            elif status == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Goal {i} reached.")
                finished_goals += 1
                break
            elif status in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                rospy.logerr(f"Goal {i} failed with status code: {status}")
                break
            rospy.sleep(1)
        
        # we only increment the goal index if we didn't approach a face, so that we return to the last goal
        if not new_face:
        #     client.cancel_goal()
        # else: 
            i += 1
        
        # wait for the face to be approached
        while new_face:
            rospy.sleep(1)

    rospy.loginfo(f"Found all faces! Reached {finished_goals}/{len(x_position)} goals.")

if __name__ == '__main__':
    rospy.init_node('hw3_goals')

    # rospy.sleep(5)

    # Move a robot a little bit to calibrate its position
    # calibrate()

    # Start moving to goals
    move_to_goal()


# roslaunch exercise3 rins_world.launch
# roslaunch exercise3 amcl_simulation.launch
# roslaunch turtlebot_rviz_launchers view_navigation.launch
# rosrun exercise3 hw3_goals.py
# or
# roslaunch exercise3 essential.launch
# rosrun exercise3 hw3_goals.py