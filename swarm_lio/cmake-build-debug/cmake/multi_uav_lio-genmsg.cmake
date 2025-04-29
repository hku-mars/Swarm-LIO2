# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "swarm_lio: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iswarm_lio:/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(swarm_lio_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_custom_target(_swarm_lio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_lio" "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" ""
)

get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_custom_target(_swarm_lio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_lio" "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" "geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:std_msgs/Header:swarm_lio/ObserveTeammate:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_custom_target(_swarm_lio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_lio" "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_custom_target(_swarm_lio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_lio" "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
)
_generate_msg_cpp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
)
_generate_msg_cpp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
)
_generate_msg_cpp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
)

### Generating Services

### Generating Module File
_generate_module_cpp(swarm_lio
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(swarm_lio_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(swarm_lio_generate_messages swarm_lio_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_cpp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_cpp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_cpp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_cpp _swarm_lio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_lio_gencpp)
add_dependencies(swarm_lio_gencpp swarm_lio_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_lio_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
)
_generate_msg_eus(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
)
_generate_msg_eus(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
)
_generate_msg_eus(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
)

### Generating Services

### Generating Module File
_generate_module_eus(swarm_lio
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(swarm_lio_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(swarm_lio_generate_messages swarm_lio_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_eus _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_eus _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_eus _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_eus _swarm_lio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_lio_geneus)
add_dependencies(swarm_lio_geneus swarm_lio_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_lio_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
)
_generate_msg_lisp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
)
_generate_msg_lisp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
)
_generate_msg_lisp(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
)

### Generating Services

### Generating Module File
_generate_module_lisp(swarm_lio
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(swarm_lio_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(swarm_lio_generate_messages swarm_lio_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_lisp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_lisp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_lisp _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_lisp _swarm_lio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_lio_genlisp)
add_dependencies(swarm_lio_genlisp swarm_lio_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_lio_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
)
_generate_msg_nodejs(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
)
_generate_msg_nodejs(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
)
_generate_msg_nodejs(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
)

### Generating Services

### Generating Module File
_generate_module_nodejs(swarm_lio
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(swarm_lio_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(swarm_lio_generate_messages swarm_lio_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_nodejs _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_nodejs _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_nodejs _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_nodejs _swarm_lio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_lio_gennodejs)
add_dependencies(swarm_lio_gennodejs swarm_lio_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_lio_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
)
_generate_msg_py(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
)
_generate_msg_py(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
)
_generate_msg_py(swarm_lio
  "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
)

### Generating Services

### Generating Module File
_generate_module_py(swarm_lio
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(swarm_lio_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(swarm_lio_generate_messages swarm_lio_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/Pose6D.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_py _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/QuadStatePub.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_py _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/States.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_py _swarm_lio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/msg/ObserveTeammate.msg" NAME_WE)
add_dependencies(swarm_lio_generate_messages_py _swarm_lio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_lio_genpy)
add_dependencies(swarm_lio_genpy swarm_lio_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_lio_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_lio
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(swarm_lio_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_lio
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(swarm_lio_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_lio
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(swarm_lio_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_lio
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(swarm_lio_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_lio
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(swarm_lio_generate_messages_py geometry_msgs_generate_messages_py)
endif()
