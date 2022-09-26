cmake_minimum_required(VERSION 3.5)
project(hiros_skeleton_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

set(msg_files
    "msg/Box.msg"
    "msg/KinematicState.msg"
    "msg/Link.msg"
    "msg/Marker.msg"
    "msg/Skeleton.msg"
    "msg/SkeletonGroup.msg"
)

set(HEADER_FILES
    include/skeletons/types.h
    include/skeletons/utils.h
)

set(SOURCE_FILES
    src/types.cpp
    src/utils.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(skeletons ${SOURCE_FILES} ${HEADER_FILES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(skeletons "${cpp_typesupport_target}")
target_include_directories(skeletons PRIVATE include)
ament_target_dependencies(skeletons rclcpp std_msgs geometry_msgs tf2)
ament_export_targets(skeletons HAS_LIBRARY_TARGET)
ament_export_libraries(skeletons)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake builtin_interfaces std_msgs geometry_msgs tf2)

install(
  DIRECTORY include/skeletons
  DESTINATION include
)

install(
  TARGETS skeletons
  EXPORT skeletons
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
