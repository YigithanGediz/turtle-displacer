cmake_minimum_required(VERSION 3.0.2)
project(autonomous_turtle)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
    actionlib
    actionlib_msgs
)



#add_service_files(
#    FILES
#    ConvertMetresToFeet.srv
#)

add_action_files(
    FILES
    GoToPosition.action
)

#add_message_files(DIRECTORY msg
#    FILES
#    SensorInformation.msg
#)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime actionlib_msgs
)

