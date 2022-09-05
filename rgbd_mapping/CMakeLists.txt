cmake_minimum_required(VERSION 2.8.3)
project(tiago_2dnav_gazebo)

find_package(catkin REQUIRED COMPONENTS
    pal_navigation_msgs
    roscpp
)

generate_messages(
   DEPENDENCIES
   pal_navigation_msgs
 )

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(interface_node src/interface.cpp)
add_dependencies(interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(interface_node
   ${catkin_LIBRARIES}
 )
 

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
