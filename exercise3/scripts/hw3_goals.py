#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def calibrate():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = -0.5
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.orientation.w = 1.0
    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Robot calibrated.")


def move_to_goal():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Goal coordinates
    x = [0.5, 3.16, 1.20, 1.05, -0.77]
    y = [-1.3, -1.11, 0.37, 1.92, 1.19]

    finished_goals = 0
    for i in range(len(x)):
        # Create a goal message
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x[i]
        goal.target_pose.pose.position.y = y[i]
        goal.target_pose.pose.orientation.w = 1.0

        # Send the goal and wait for the result
        client.send_goal(goal)

        # Print the status of the goal
        while not rospy.is_shutdown():

            # to-do: check if face is detected
            # call approch_face() if face is detected

            status = client.get_state()
            if status == GoalStatus.ACTIVE:
                rospy.loginfo(f"Goal {i} is being processed.")
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

        # Wait for the result and then print the status
        # client.wait_for_result()
        # if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        #     rospy.loginfo(f"Goal [{x}, {y}] reached.")
        # else:
        #     rospy.loginfo(f"Failed to reach goal [{x}, {y}].")

    rospy.loginfo(f"Final goal reached! Reached {finished_goals}/{len(x)} goals.")

if __name__ == '__main__':
    rospy.init_node('hw3_goals')

    # Move a robot a little bit to calibrate its position
    calibrate()

    # Start moving to goals
    move_to_goal()


# roslaunch exercise3 rins_world.launch
# roslaunch exercise3 amcl_simulation.launch
# roslaunch turtlebot_rviz_launchers view_navigation.launch
# rosrun exercise3 hw3_goals.py
# or
# roslaunch exercise3 essential.launch
# rosrun exercise3 hw3_goals.py