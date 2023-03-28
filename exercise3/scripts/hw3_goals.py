#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def move_to_goal():
    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Goal coordinates
    x = [0.10408163070678711, 3.16398286819458, 1.20012366771698, 1.051872730255127, -0.7761532068252563]
    y = [-1.7393693923950195, -1.1116236448287964, 0.3751080334186554, 1.928648591041565, 1.190643310546875]

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
    move_to_goal()


# roslaunch exercise3 rins_world.launch
# roslaunch exercise3 amcl_simulation.launch
# roslaunch turtlebot_rviz_launchers view_navigation.launch
# rosrun exercise3 hw3_goals.py
# or
# roslaunch exercise3 essential.launch
# rosrun exercise3 hw3_goals.py