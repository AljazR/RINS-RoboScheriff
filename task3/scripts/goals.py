#!/usr/bin/python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped
from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from task3.srv import Dialogue, DialogueResponse
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

global sound_client
global arm_command_pub
new_face = False

def precise_parking(pose: Pose):
    # Create a SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Get a global costmap 
    cost_map = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
    resolution = cost_map.info.resolution
    origin = cost_map.info.origin.position
    width = cost_map.info.width

    # Check if pose is valid
    if cost_map.data[int((pose.position.x - origin.x) / resolution) + int((pose.position.y - origin.y) / resolution) * width] < 0.4:
        print("Pose is invalid.")
        # Find closest valid pose
        new_x = 0
        new_y = 0
        closest_distance = 1000
        for i in range(len(cost_map.data)):
            if cost_map.data[i] < 0.4:
                temp_x = origin.x + (i % width) * resolution
                temp_y = origin.y + (i // width) * resolution
                distance = math.sqrt((pose.position.x - temp_x)**2 + (pose.position.y - temp_y)**2)
                if distance < closest_distance:
                    closest_distance = distance
                    new_x = temp_x
                    new_y = temp_y
        # Update pose
        pose.position.x = new_x
        pose.position.y = new_y
        print(f"New pose: [{new_x:.3f}, {new_y:.3f}]")
    else:
        print("Pose is valid.")
    
    # Go to the pose under the ring
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = pose.position.x
    goal.target_pose.pose.position.y = pose.position.y
    goal.target_pose.pose.orientation.w = 0
    goal.target_pose.pose.orientation.z = 1
    client.send_goal(goal)
    client.wait_for_result()

    # Extend the arm when the pose is reached
    global arm_command_pub
    arm_command_pub.publish("extend")
    rospy.sleep(2)

    # Get current location of the robot
    robot_location = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

    # Spin on the spot
    w = [ 0.7, 1, 0.7]
    z = [-0.7, 0, 0.7]
    for i in range(3):
        goal.target_pose.pose.position.x = robot_location.pose.pose.position.x
        goal.target_pose.pose.position.y = robot_location.pose.pose.position.y
        goal.target_pose.pose.orientation.z = z[i]
        goal.target_pose.pose.orientation.w = w[i]
        client.send_goal(goal)
        client.wait_for_result()        

    # TODO: find the ring on the floor and park in it
    # pose = rospy.wait_for_message('/arm_ring_pose', Pose)
    # goal.target_pose.pose.position.x = pose.position.x
    # goal.target_pose.pose.position.y = pose.position.y
    # goal.target_pose.pose.orientation.z = z[0]
    # goal.target_pose.pose.orientation.w = w[0]
    # client.send_goal(goal)
    # client.wait_for_result()

def approach_cylinder(pose: Pose):

    # TODO

    global sound_client
    sound_client.say("I'm taking you to the prison.")
    rospy.sleep(3)

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
        print(f"Approaching face: [{x:.3f}, {y:.3f}].")

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

        # When face is approached, open the dialogue box
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            # Wait for the service to become available
            rospy.wait_for_service('dialogue_box')
            try:
                # Create a callable for the service
                dialogue_box = rospy.ServiceProxy('dialogue_box', Dialogue)          

                # Create a request message
                req = Dialogue()
                req.start = True
                
                # Call the service
                res = dialogue_box(req.start)
                print(f"Service call succeeded with response: {res.success}")

            except rospy.ServiceException as e:
                print('\033[93m' + f"Service call to dialog_box failed: {e}" + '\033[0m')

        else:
            print(f"Failed to approach face.")

    new_face = False

def move_to_goals():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    rospy.Subscriber('face_markers', MarkerArray, approach_face)

    # Goal coordinates
    x_position =    [-0.64, -1.27, -0.07,  0.15,  0.14,  0.89,  1.92,  2.20,  3.65,  2.00,  1.11,  1.57,  2.58,  2.58,  1.82,  0.95, -0.03, -0.83]
    y_position =    [ 0.95,  0.05, -0.33, -0.78, -1.33, -1.39, -1.11, -0.81, -0.80,  0.67,  1.45,  0.35,  1.10,  2.06,  2.51,  2.55,  2.67,  1.69]
    z_orientation = [-0.98,  0.86, -0.02, -0.99, -0.46,  0.54, -0.58, -0.66,  1.00,  0.77,  0.15, -0.99,  0.93,  0.16, -0.73,  0.70, -0.60, -0.24]
    w_orientation = [ 0.21,  0.51,  0.99,  0.08,  0.89,  0.84,  0.81,  0.75,  0.00,  0.63,  0.99,  0.13,  0.37,  0.99,  0.68,  0.71,  0.80,  0.99]

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
        while not rospy.is_shutdown() and not new_face:
            status = client.get_state()
            if status == GoalStatus.SUCCEEDED:
                print(f"Goal {i} reached.")
                break
            elif status in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                print('\033[93m' + f"Goal {i} failed with status code: {status}" + '\033[0m')
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

    print(f"Finished designated goals.")

def get_colors():
    data = rospy.wait_for_message('/colors', String, timeout=None)
    colors = data.data.split()
    cylinder1 = colors[0]
    cylinder2 = colors[1]
    ring = colors[2]
    return cylinder1, cylinder2, ring

def get_cylinder_pose(cylinder1, cylinder2):
    marker_array = rospy.wait_for_message('/cylinder_markers', MarkerArray, timeout=None)

    for marker in marker_array.markers:
        if marker.color.r == 1 and marker.color.g == 0 and marker.color.b == 0 and cylinder1 == "red" \
        or marker.color.r == 0 and marker.color.g == 1 and marker.color.b == 0 and cylinder1 == "green" \
        or marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 1 and cylinder1 == "blue" \
        or marker.color.r == 1 and marker.color.g == 1 and marker.color.b == 0 and cylinder1 == "yellow":
            return cylinder1, marker.pose

    for marker in marker_array.markers:
        if marker.color.r == 1 and marker.color.g == 0 and marker.color.b == 0 and cylinder2 == "red" \
        or marker.color.r == 0 and marker.color.g == 1 and marker.color.b == 0 and cylinder2 == "green" \
        or marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 1 and cylinder2 == "blue" \
        or marker.color.r == 1 and marker.color.g == 1 and marker.color.b == 0 and cylinder2 == "yellow":
            return cylinder2, marker.pose

    return None, None

def get_ring_pose(ring):
    marker_array = rospy.wait_for_message('/ring_markers', MarkerArray, timeout=None)

    for marker in marker_array.markers:
        if marker.color.r == 1 and marker.color.g == 0 and marker.color.b == 0 and ring == "red" \
        or marker.color.r == 0 and marker.color.g == 1 and marker.color.b == 0 and ring == "green" \
        or marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 1 and ring == "blue" \
        or marker.color.r == 0 and marker.color.g == 0 and marker.color.b == 0 and ring == "black":
            return marker.pose
    
    return None

def main():
    # Initialize the node
    rospy.init_node('goals')

    # Retract the arm
    global arm_command_pub
    arm_command_pub = rospy.Publisher('/arm_command', String, queue_size=10)
    rospy.sleep(1)
    arm_command_pub.publish("retract")

    # Set sound clinet
    global sound_client
    sound_client = SoundClient()
    rospy.sleep(3)

    # Go through all goals
    move_to_goals()

    # Get the ring and cylinders colors
    cylinder1, cylinder2, ring = get_colors()
    if ring is None or cylinder1 is None or cylinder2 is None:
        print('\033[93m' + f"Not the right face detected. No info about the colors." + '\033[0m')
        return
    print(f"Colors form dialogue: ring: {ring}, cylinder 1: {cylinder1}, cylinder 2: {cylinder2}.")

    '''
    # TODO: set latch=True in publisher for cylinders' MarkerArray. It keeps the last message posted until a new message arrives - we need it for wait_for_message function to work
    # Get a cylinder pose
    # cylinder, cylinder_pose = get_cylinder_pose(cylinder1, cylinder2)
    # if cylinder_pose is None:
    #     print(f"Both cylinders were not detected. Skipping cylinder approach.")
    # else:
    #     print(f"Approaching {cylinder} cylinder with position: [{cylinder_pose.position.x:.3f}, {cylinder_pose.position.y:.3f}]")
    #     # Approach the cylinder
    #     approach_cylinder(cylinder_pose)
    '''

    # Get a ring pose
    # ring = "blue"
    ring_pose = get_ring_pose(ring)
    if ring_pose is None:
        print('\033[93m' + f"{ring[0].upper() + ring[1:]} ring was not detected. Skipping precise parking." + '\033[0m')
    else:
        print(f"Precise parking under {ring} ring with pose: [{ring_pose.position.x:.3f}, {ring_pose.position.y:.3f}]")
        # Precise parking
        precise_parking(ring_pose)

    # Wave with robot arm and say "I'm done!"
    sound_client.say("I'm done!")
    arm_command_pub.publish("wave")

if __name__ == '__main__':
    main()


# Running task3:
# catkin_make
# roslaunch task3 essential_final.launch
# rosrun task3 dialogue_box.py 
# rosrun task3 face_localizer_dnn.py 
# rosrun task3 detect_rings.py 
# rosrun task3 goals.py
