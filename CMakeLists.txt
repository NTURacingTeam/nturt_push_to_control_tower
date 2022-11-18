cmake_minimum_required(VERSION 3.0.2)
project(nturt_push_to_control_tower)

find_package(catkin REQUIRED
        gps_common
        nturt_ros_interface
        roscpp
        std_msgs
)

find_package(Boost REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS gps_common nturt_ros_interface roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_executable(nturt_push_to_control_tower
        src/nturt_push_to_control_tower_core.cpp
        src/nturt_push_to_control_tower_node.cpp
)

target_link_libraries(nturt_push_to_control_tower
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_dependencies(nturt_push_to_control_tower
        ${catkin_EXPORTED_TARGETS}
)
