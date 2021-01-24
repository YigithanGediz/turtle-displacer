#!/usr/bin/env python

import rospy
from autonomous_turtle.msg import GoToPositionAction,GoToPositionFeedback,GoToPositionResult
import actionlib
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt,pow,atan,pi

class Go_To_Position_Class:
    _feedback = GoToPositionFeedback()
    _result = GoToPositionResult()

    def __init__(self, actionname):
        # actionname will be used by clients to connect to the server

        self.r = 30
        self.rate = rospy.Rate(self.r)
        self._action_name = actionname

        self._as = actionlib.SimpleActionServer(self._action_name, GoToPositionAction,execute_cb=self.execute_cb, auto_start=False)

        self.pose = Pose()
        self.vel = Twist()

        self._as.start()

        rospy.loginfo("Go to position action server has started")

    def execute_cb(self, goal):
        # I defined these stuff here because I had to define them in the scope of the goal.

        self.goal = goal
        self.pose_subscriber = rospy.Subscriber("/{}/pose".format(self.goal.turtlename), Pose, self.update_pose)
        self.vel_publisher = rospy.Publisher("/{}/cmd_vel".format(self.goal.turtlename), Twist, queue_size=self.r + 10)

        rospy.loginfo("{} is starting to move".format(self.goal.turtlename))
        success = self.move()

        if success:
            self._result.result_message = "Successfully reached to the goal"
            rospy.loginfo("{}: Succeeded".format(self._action_name))
            self._as.set_succeeded(result=self._result)
        else:
            self._result.result_message = "Robot Failed"
            rospy.loginfo("{}: Failed".format(self._action_name))
            self._as.set_preempted(result=self._result)

    def update_pose(self, data):
        # Callback function to receive the current position
        self.pose = data

    def euclidean_distance(self, initial, final):
        return sqrt(pow((initial[0]-final[0]),2) + pow((initial[1]-final[1]),2))

    def angle_between(self, coord1, coord2):
        # Angle between two coordinates in radians
        try:
            value = atan((coord1[1]-coord2[1])/(coord1[0]-coord2[0]))
        except ZeroDivisionError:
            value = pi/2

        return value

    def rotate(self,goal_angle):
        # Rotate the turtle until theta passes goal_angle

        angular_vel = rospy.get_param("angular_velocity", 2.5)
        tolerance = rospy.get_param("rotation_tolerance", 0.05)

        delta = self.pose.theta - goal_angle

        # If the robot is already steering at the goal
        if abs(delta) <= tolerance:
            return

        # To shorten the amount of rotation, robot's direction of rotation will depend on the sign
        # of the angular difference
        if delta > 0:
            angular_vel *= -1


        vel_msg = Twist()

        vel_msg.angular.x = vel_msg.angular.y = vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0
        vel_msg.angular.z = angular_vel


        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("{}: Preempted".format(self._action_name))
                self._as.set_preempted()
                break

            current_angle = self.pose.theta
            if delta > 0 and current_angle - goal_angle > tolerance:
                self.vel_publisher.publish(vel_msg)

            elif delta <= 0 and current_angle - goal_angle <= -1*tolerance:
                self.vel_publisher.publish(vel_msg)

            else:
                return

            self.rate.sleep()

    def forward_or_back(self, goal, initial):
        # Since arctan returns values just between plus and minus pi/2, we need to decide if the goal
        # is located at behind our robot or in front of our robot
        # Vec is simply the location of the goal relative to the robot
        # if vec is in the second or third quadrant, the goal is at the back
        vec = (goal[0]-initial[0] , goal[1]-initial[1])
        if (vec[0] <= 0 and vec[1] >= 0) or (vec[0] <= 0 and vec[1] <= 0):
            return -1

        return 1


    def move(self):
        # The function that will make the turtle move
        # Initializing the parametes

        goal_coords = (self.goal.x, self.goal.y)
        initial_coords = (self.pose.x, self.pose.y)

        # Sometimes pose returns 0 as position values at first call
        while initial_coords == (0.0, 0.0) and not rospy.is_shutdown():
            initial_coords = (self.pose.x, self.pose.y)

        # The angle between current and aimed position
        goal_angle = self.angle_between(goal_coords, initial_coords)

        # Rotating the turtle to make it look forward to the goal
        self.rotate(goal_angle)

        distance = self.euclidean_distance(initial_coords,goal_coords)
        vel_msg = Twist()

        vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = vel_msg.linear.z = vel_msg.linear.y = 0
        vel_msg.linear.x = rospy.get_param("translational_velocity", 5) * self.forward_or_back(goal_coords,initial_coords)
        tolerance = rospy.get_param("translational_tolerance", 0.03)

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("{}: Preempted".format(self._action_name))
                return False

            current_loc = (self.pose.x, self.pose.y)
            if current_loc == (0.0, 0.0):
                return False

            if distance - self.euclidean_distance(current_loc, initial_coords) >= tolerance:
                self.vel_publisher.publish(vel_msg)
                self._feedback.current_x = self.pose.x
                self._feedback.current_y = self.pose.y
                self._as.publish_feedback(self._feedback)

            else:
                vel_msg.linear.x = 0
                self.vel_publisher.publish(vel_msg)
                return True

            self.rate.sleep()

        return False

if __name__ == "__main__":
    # Initialize a ROS node for this action server
    rospy.init_node("go_to_position_server", anonymous=False)

    server = Go_To_Position_Class(rospy.get_name())
    rospy.spin()
