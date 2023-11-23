cmake_minimum_required(VERSION 3.0.2)
project(ev_main)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)


add_message_files(FILES CmdMsg.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ev_main_node src/ev_main_node.cpp)
add_dependencies(ev_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ev_main_node ${catkin_LIBRARIES})
