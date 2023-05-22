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

parking = False

def precise_parking(pose: Pose):
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


def move_to_goal():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pub = rospy.Publisher('/arm_command', String, queue_size=10)
    pub.publish("retract")

    rospy.Subscriber('green_ring_pose', Pose, precise_parking)

    # Goal coordinates
    x_position =    [-0.19,  1.98,  2.12,  3.23,  1.00, -0.35] #  0.00,  0.41,
    y_position =    [-0.39,  2.46,  0.97, -0.58, -1.32,  0.00] # -1.00,  2.47,
    z_orientation = [ 0.79, -0.02, -0.70, -0.98,  0.05,  1.00] # -0.03, -0.37,
    w_orientation = [ 0.62,  1.00,  0.70,  0.22,  1.00,  0.03] #  1.00,  0.93,

    # x_position =    [-0.48,  0.32,  2.43,  3.61,  0.46]
    # y_position =    [-0.17,  1.91,  2.05, -1.30, -1.20]
    # z_orientation = [ 0.71, -0.35, -0.99, -0.99,  0.00]
    # w_orientation = [ 0.70,  0.93,  0.03,  0.06,  1.00]

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
        while not rospy.is_shutdown() and not parking:
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
        

        # we only increment the goal index if we aren't parking, so that we return to the last goal
        if parking:
            client.cancel_goal()
        else: 
            i += 1

        # wait for the parking to finish
        while parking:
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('hw3_goals')

    # Start moving to goals
    move_to_goal()