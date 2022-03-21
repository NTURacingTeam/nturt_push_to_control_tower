cmake_minimum_required(VERSION 3.0.2)
project(nturt_push_to_control_tower)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -g")

find_package(catkin REQUIRED rostime roscpp rosconsole roscpp_serialization nav_msgs can_msgs)
find_package(nturt_can_parser REQUIRED)

catkin_package(CATKIN_DEPENDS nturt_can_parser)

include_directories(inc ${catkin_INCLUDE_DIRS} ${nturt_can_parser_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${nturt_logger_INCLUDE_DIRS} ${nturt_can_parser_INCLUDE_DIRS})

add_executable(nturt_push_to_control_tower src/nturt_push_to_control_tower_core.cpp src/nturt_push_to_control_tower_node.cpp)
target_link_libraries(nturt_push_to_control_tower ${catkin_LIBRARIES} ${nturt_can_parser_LIBRARIES})
add_dependencies(nturt_push_to_control_tower ${catkin_EXPORTED_TARGETS})

install(TARGETS nturt_push_to_control_tower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
