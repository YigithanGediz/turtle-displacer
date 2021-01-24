# turtle-displacer
It is a ROS action which you can specify the coordinates that you want to displace your turtle to in turtlesim.
With this action, it is possible to pass the desired x and y coordinates of a turtle and action takes care of it.
Turtle will first rotate until it looks forward to the goal and then start the movement.
It is also possible to control different turtles by specifying different turtlenames but you need to start a unique server for each one.
There are also some parameters that the user can set via rosparam. These are translational_velocity, rotational_velocity, translational_tolerance and rotational_tolerance. When the difference between the desired position and the current position fall below the tolerance, turtle will stop. Same thing happens for rotational_tolerance.
