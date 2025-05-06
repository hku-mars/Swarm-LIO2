cmake_minimum_required(VERSION 2.8.3)
project(udp_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        roslib
        rosbag
        swarm_msgs
        rosfmt
        )

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
        ../../../devel/include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(udp_online
        src/udp_online.cpp
        )
target_link_libraries(udp_online
        ${catkin_LIBRARIES}
        )
