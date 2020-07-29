cmake_minimum_required(VERSION 3.0)
project(skeleton_msgs)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS geometry_msgs
                        message_generation
                        std_msgs)

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# Generate messages in the 'msg' folder
add_message_files(FILES
                  Box.msg
                  Keypoint.msg
                  KeypointGroup.msg
                  Point.msg
                  Skeleton.msg
                  SkeletonGroup.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  geometry_msgs
                  std_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(CATKIN_DEPENDS
               geometry_msgs
               message_runtime
               std_msgs
               LIBRARIES
               ${PROJECT_NAME}_types
               INCLUDE_DIRS
               include)

# ##############################################################################
# Build ##
# ##############################################################################
set(${PROJECT_NAME}_SOURCES src/types.cpp src/utils.cpp)

add_library(${PROJECT_NAME}_types SHARED ${${PROJECT_NAME}_SOURCES})

# Specify additional locations of header files
target_include_directories(${PROJECT_NAME}_types
                           PUBLIC include ${catkin_INCLUDE_DIRS})

add_dependencies(${PROJECT_NAME}_types ${PROJECT_NAME}_generate_messages_cpp)

# ##############################################################################
# Create fake target for common ROS file extensions
# ##############################################################################

file(GLOB_RECURSE
     FILES_LIST
     "*.msg"
     "*.srv"
     "*.xml")

add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS ${PROJECT_NAME}_types
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/skeletons/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
