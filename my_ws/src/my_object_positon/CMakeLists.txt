cmake_minimum_required(VERSION 2.8.3)
project(my_object_position)

# Compile as C++11 (ROS Noetic supports C++11 by default)
add_compile_options(-std=c++11)

# Find necessary packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  yolov8_ros_msgs  # Add darknet_ros_msgs here
)

# Add Boost package if needed (if you're not using Boost, you can remove this line)
find_package(Boost REQUIRED)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime tf sensor_msgs cv_bridge image_transport geometry_msgs yolov8_ros_msgs  # Add darknet_ros_msgs as a dependency
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/usr/include/eigen3"  # Eigen3 is often used for matrix operations
)

# Link directories (usually not needed unless you have specific libraries to link)
link_directories(${catkin_LIB_DIRS})

# Add executable
add_executable(my_object_position src/my_object_position.cpp)

# Link libraries to your executable
target_link_libraries(my_object_position
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# If you have any message or service files, ensure they are generated here (uncomment if used)
# add_message_files(
#   FILES
#   YourMessage.msg
# )

# Uncomment the following to generate messages if you have message files
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
