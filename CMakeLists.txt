cmake_minimum_required(VERSION 3.0)
project(skeleton_msgs)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS message_generation
                        std_msgs
                        geometry_msgs)

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# Generate messages in the 'msg' folder
add_message_files(FILES
                  Keypoint.msg
                  Rectangle.msg
                  KeypointGroup.msg
                  Skeleton.msg
                  SkeletonGroup.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)

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
               message_runtime
               std_msgs
               geometry_msgs)

catkin_package(LIBRARIES ${PROJECT_NAME}_types
               INCLUDE_DIRS include)

# ##############################################################################
# Build ##
# ##############################################################################
set(${PROJECT_NAME}_SOURCES
    src/types.cpp
    src/utils.cpp)

add_library(${PROJECT_NAME}_types SHARED ${${PROJECT_NAME}_SOURCES})

# Specify additional locations of header files
target_include_directories(${PROJECT_NAME}_types PUBLIC
    include
    ${catkin_INCLUDE_DIRS})

# Display additional files in qtcreator
file(GLOB_RECURSE
     FILES_LIST
     "*.msg"
     "*.srv"
     "*.xml")

add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# ##############################################################################
# Install ##
# ##############################################################################

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination install(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables and/or libraries for installation install(TARGETS
# ${PROJECT_NAME} ${PROJECT_NAME}_node ARCHIVE DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )
