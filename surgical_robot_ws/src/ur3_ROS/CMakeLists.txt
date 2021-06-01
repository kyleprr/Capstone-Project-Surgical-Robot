cmake_minimum_required(VERSION 2.8.3)
project(surgical_robot)

find_package(catkin REQUIRED COMPONENTS 
    gazebo_msgs 
    roscpp 
    geometry_msgs 
    std_srvs 
    std_msgs 
    message_generation
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})