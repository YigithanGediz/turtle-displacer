#!/usr/bin/env python

from autonomous_turtle.msg import GoToPositionAction,GoToPositionGoal,GoToPositionActionResult
import actionlib
import rospy
import sys

def go_to_position_client(name, desired_x, desired_y):
    # Creating the client
    # Client takes command line arguments
    # Rotational

    client = actionlib.SimpleActionClient("go_to_position_server", GoToPositionAction)

    # Waits until the action server has stated up and started listening the goals
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    # Creates a goal to send the action server
    goal = GoToPositionGoal(x=desired_x, y=desired_y, turtlename= name)

    # Sends the goal to the action server
    client.send_goal(goal)

    rospy.loginfo("Goal has been sent to the action server")
    client.wait_for_result()

    return client.get_result()

if __name__ == "__main__":
    try:
        if len(sys.argv) != 4:
            print("usage: rosrun package_name go_to_position_ac.py turtlename desired_x desired_y")
        # Initializes a rospy node so that the go_to_position_client can
        # publish and subscribe over ROS
        else:
            rospy.init_node("go_to_position_client", anonymous=False)
            turtlename, x, y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])

            if x < 0 or y > 11.1:
                print("desired_x can't be lower than 0 and desired_y can't exceed 11.1")
                sys.exit()

            # To prevent the bugs that might occur at boundaries
            if x == 0:
                x += 0.2
            if y == 11.1:
                y -= 0.2

            result = go_to_position_client(turtlename, x, y)
            rospy.loginfo(result.result_message)
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
