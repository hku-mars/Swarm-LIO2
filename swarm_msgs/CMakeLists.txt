cmake_minimum_required(VERSION 2.8.3)
project(swarm_msgs)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        message_generation
        )
add_message_files(
        DIRECTORY msg
        FILES
        GlobalExtrinsic.msg
        GlobalExtrinsicStatus.msg
        QuadStatePub.msg
        ObserveTeammate.msg
        TeamStatus.msg
        TeammateInfo.msg
        SpatialTemporalOffset.msg
        SpatialTemporalOffsetStatus.msg
        ConnectedTeammateList.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
)


include_directories(
        ../../../devel/include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)
