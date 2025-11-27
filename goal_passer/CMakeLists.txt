cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(goal_passer)

# Add -fsigned-char flag for ARM compatibility
# Only apply on ARM architecture to avoid potential issues on x86/x64
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")
endif()

set(goal_passer_ROS_DEPS roscpp costmap_2d pluginlib nav_core)

find_package(catkin REQUIRED COMPONENTS ${goal_passer_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES goal_passer
  CATKIN_DEPENDS ${goal_passer_ROS_DEPS}
)

add_library(goal_passer src/goal_passer.cpp)
target_link_libraries(goal_passer ${catkin_LIBRARIES})

install(TARGETS goal_passer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
