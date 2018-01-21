cmake_minimum_required(VERSION 2.8.3)
project(compass_test)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf)

catkin_package(CATKIN_DEPENDS roscpp rospy)

include_directories(${catkin_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)

add_executable(tf_publish src/tf_publish.cpp)
target_link_libraries(tf_publish ${catkin_LIBRARIES})
