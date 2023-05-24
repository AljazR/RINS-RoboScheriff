#!/usr/bin/python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

new_face = False
ring_markers = None
cylinder_markers = None

def get_ring_markers(marker_array: MarkerArray):
    global ring_markers
    ring_markers = marker_array.markers
    return

def get_cylinder_markers(marker_array: MarkerArray):
    global cylinder_markers
    cylinder_markers = marker_array.markers
    return

def precise_parking(pose: Pose):
    # TODO: this is just a draft

    global parking
    parking = True
    
    pub = rospy.Publisher('/arm_command', String, queue_size=10)
    pub.publish("extend")

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # rospy.Subscriber('arm_ring_pose', Pose, ??)
    
    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = pose.position.x
    goal.target_pose.pose.position.y = pose.position.y

    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()

    pub.publish("retract")
    parking = False

def approach_cylinder(pose: Pose):
    # TODO

    # Say: "I'm taking you to the prison."
    return

def approach_face(marker_array: MarkerArray):
    # if the new marker is black, then it is not a face but a wanted poster
    if marker_array.markers[-1].color.r == 1:
        return

    global new_face
    new_face = True
    
    # Location of the face
    face_position = marker_array.markers[-1].pose.position
    face_orientation = marker_array.markers[-1].pose.orientation

    # Location of the robot
    robot_location = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    robot_position = robot_location.pose.pose.position
    robot_orientation = robot_location.pose.pose.orientation

    # Find closest point facing the face
    new_x = [face_position.x + 0.5, face_position.x - 0.5, face_position.x, face_position.x]
    new_y = [face_position.y, face_position.y, face_position.y + 0.5, face_position.y - 0.5]
    new_w = [0, 1, 0.7, 0.7]
    new_z = [1, 0, -0.7, 0.7]
    x = y = w = z = None
    min_dist = 1000
    for i in range(4):
        dist = math.sqrt((new_x[i] - robot_position.x)**2 + (new_y[i] - robot_position.y)**2)
        if dist < min_dist:
            min_dist = dist
            x = new_x[i]
            y = new_y[i]
            w = new_w[i]
            z = new_z[i]

    if x is not None:
        rospy.loginfo(f"Approaching point [{x:.3f}, {y:.3f}].")

        # Send the robot to the face
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
            sound_client.say("Hello! I detected your face!")
            rospy.sleep(4)
        else:
            rospy.loginfo(f"Failed to approach face.")

    new_face = False

def move_to_goals():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    rospy.Subscriber('face_markers', MarkerArray, approach_face)

    # Goal coordinates
    x_position =    [-0.64, -1.27, -0.07,  0.15,  0.14,  0.89,  1.92,  2.87,  2.00,  1.11,  1.57,  2.58,  2.58,  1.82,  0.95, -0.03, -0.83, -1.54, -1.45]
    y_position =    [ 0.95,  0.05, -0.33, -0.78, -1.33, -1.39, -1.11, -0.81,  0.67,  1.45,  0.35,  1.10,  2.06,  2.51,  2.55,  2.67,  1.69,  2.70,  1.68]
    z_orientation = [-0.98,  0.86, -0.02, -0.99, -0.46,  0.54, -0.58, -0.66,  0.77,  0.15, -0.99,  0.93,  0.16, -0.73,  0.70, -0.60, -0.24,  0.07, -0.76]
    w_orientation = [ 0.21,  0.51,  0.99,  0.08,  0.89,  0.84,  0.81,  0.75,  0.63,  0.99,  0.13,  0.37,  0.99,  0.68,  0.71,  0.80,  0.99,  0.99,  0.65]

    i = 0
    while i < len(x_position):
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
                break
            elif status in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                rospy.logerr(f"Goal {i} failed with status code: {status}")
                break
            rospy.sleep(1)
        
        # we only increment the goal index if we didn't approach a face, so that we return to the last goal
        if new_face:
            client.cancel_goal()
        else: 
            i += 1
        
        # Wait for the face to be approached
        while new_face:
            rospy.sleep(1)

    rospy.loginfo(f"Finished designated goals.")

def get_dialogue_info():
    # TODO: connect to the service dialogue_box
    return [["blue", "red"], "red"]


def main():
    # Initialize the node
    rospy.init_node('goals')

    # Retract the arm
    arm_command_pub = rospy.Publisher('/arm_command', String, queue_size=10)
    rospy.sleep(1)
    arm_command_pub.publish("retract")

    # Start ring and cylinder detection
    rospy.Subscriber('ring_markers', MarkerArray, get_ring_markers)
    # rospy.Subscriber('cylinder_markers', MarkerArray, get_cylinder_markers)

    # Go through all goals
    move_to_goals()

    # Check if at least one right color cylinder and a ring were detected
    dialogue_info = get_dialogue_info()
    if True: # Colors are in marker arrays
        a = 1
    else: # Post error message
        rospy.logerr(f"Not the right ring or cylinder detected.")
        # Can run move_to_goals() again?
        return

    # Approach the cylinder
    # TODO
    # approach_cylinder(cylinder_pose)

    # Precise park
    # TODO
    # precise_park(ring_pose)
    
    # Wave with robot arm and say "I'm done!"
    # TODO

if __name__ == '__main__':
    main()