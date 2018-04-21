# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localisation: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/indigo/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localisation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv" NAME_WE)
add_custom_target(_localisation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localisation" "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv" "geometry_msgs/Vector3:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(localisation
  "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localisation
)

### Generating Module File
_generate_module_cpp(localisation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localisation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localisation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localisation_generate_messages localisation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv" NAME_WE)
add_dependencies(localisation_generate_messages_cpp _localisation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localisation_gencpp)
add_dependencies(localisation_gencpp localisation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localisation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(localisation
  "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localisation
)

### Generating Module File
_generate_module_lisp(localisation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localisation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localisation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localisation_generate_messages localisation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv" NAME_WE)
add_dependencies(localisation_generate_messages_lisp _localisation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localisation_genlisp)
add_dependencies(localisation_genlisp localisation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localisation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(localisation
  "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localisation
)

### Generating Module File
_generate_module_py(localisation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localisation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localisation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localisation_generate_messages localisation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dark_knight/809_ROS/qual_2b/src/localisation/srv/request_logical_pose.srv" NAME_WE)
add_dependencies(localisation_generate_messages_py _localisation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localisation_genpy)
add_dependencies(localisation_genpy localisation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localisation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localisation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localisation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(localisation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(localisation_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(localisation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(localisation_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localisation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localisation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(localisation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(localisation_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(localisation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(localisation_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localisation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localisation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localisation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(localisation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(localisation_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(localisation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(localisation_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
