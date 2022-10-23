cmake_minimum_required(VERSION 3.0.2)
project(nturt_push_to_control_tower)

find_package(catkin REQUIRED
        rostime
        roscpp
        rosconsole
        roscpp_serialization
        nav_msgs
        can_msgs
        nturt_ros_interface
        std_msgs
)

find_package(nturt_can_parser REQUIRED)
find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS nturt_can_parser)

include_directories(
        inc
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${nturt_can_parser_INCLUDE_DIRS}
)

add_executable(nturt_push_to_control_tower
        src/nturt_push_to_control_tower_core.cpp
        src/nturt_push_to_control_tower_node.cpp
)

target_link_libraries(nturt_push_to_control_tower
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${nturt_can_parser_LIBRARIES}
)

add_dependencies(nturt_push_to_control_tower
        ${catkin_EXPORTED_TARGETS}
)
