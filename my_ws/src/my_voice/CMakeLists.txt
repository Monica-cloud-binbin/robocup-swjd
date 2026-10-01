cmake_minimum_required(VERSION 3.0.2)
project(my_voice)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  sound_play
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(voice_server src/voice_server.cpp)
target_link_libraries(voice_server ${catkin_LIBRARIES})

add_executable(simple_client src/simple_client.cpp)
target_link_libraries(simple_client ${catkin_LIBRARIES})

install(TARGETS voice_server simple_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
