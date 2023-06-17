cmake_minimum_required(VERSION 3.8)
project(nturt_push_to_control_tower)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nturt_can_config REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
)

# libraries
# library: nturt_push_to_control_tower
add_library(${PROJECT_NAME} SHARED
        src/push_to_control_tower.cpp
)
target_link_libraries(nturt_push_to_control_tower
        ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
        can_msgs
        geometry_msgs
        nturt_can_config
        rclcpp
        rclcpp_components
        sensor_msgs
)
# register to node components
# node component: PushToControlTower
rclcpp_components_register_nodes(nturt_push_to_control_tower "PushToControlTower")
set(node_plugins "${node_plugins}PushToControlTower;$<TARGET_FILE:nturt_push_to_control_tower>\n")

# applications
# executable: nturt_push_to_control_tower_node
add_executable(${PROJECT_NAME}_node
        app/push_to_control_tower_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

# install launch scripts
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        adment_cmake
        can_msgs
        geometry_msgs
        nturt_can_config
        rclcpp
        rclcpp_components
        sensor_msgs
)

ament_package()
