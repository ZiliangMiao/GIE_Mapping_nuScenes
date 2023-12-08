# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "GIE: 1 messages, 0 services")

set(MSG_I_FLAGS "-IGIE:/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(GIE_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_custom_target(_GIE_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "GIE" "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(GIE
  "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/GIE
)

### Generating Services

### Generating Module File
_generate_module_cpp(GIE
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/GIE
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(GIE_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(GIE_generate_messages GIE_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_dependencies(GIE_generate_messages_cpp _GIE_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(GIE_gencpp)
add_dependencies(GIE_gencpp GIE_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS GIE_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(GIE
  "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/GIE
)

### Generating Services

### Generating Module File
_generate_module_eus(GIE
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/GIE
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(GIE_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(GIE_generate_messages GIE_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_dependencies(GIE_generate_messages_eus _GIE_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(GIE_geneus)
add_dependencies(GIE_geneus GIE_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS GIE_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(GIE
  "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/GIE
)

### Generating Services

### Generating Module File
_generate_module_lisp(GIE
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/GIE
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(GIE_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(GIE_generate_messages GIE_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_dependencies(GIE_generate_messages_lisp _GIE_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(GIE_genlisp)
add_dependencies(GIE_genlisp GIE_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS GIE_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(GIE
  "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/GIE
)

### Generating Services

### Generating Module File
_generate_module_nodejs(GIE
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/GIE
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(GIE_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(GIE_generate_messages GIE_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_dependencies(GIE_generate_messages_nodejs _GIE_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(GIE_gennodejs)
add_dependencies(GIE_gennodejs GIE_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS GIE_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(GIE
  "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/GIE
)

### Generating Services

### Generating Module File
_generate_module_py(GIE
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/GIE
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(GIE_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(GIE_generate_messages GIE_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/MOS_Projects/GIE_ws/src/GIE-mapping/msg/CostMap.msg" NAME_WE)
add_dependencies(GIE_generate_messages_py _GIE_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(GIE_genpy)
add_dependencies(GIE_genpy GIE_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS GIE_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/GIE)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/GIE
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(GIE_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/GIE)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/GIE
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(GIE_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/GIE)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/GIE
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(GIE_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/GIE)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/GIE
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(GIE_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/GIE)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/GIE\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/GIE
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(GIE_generate_messages_py std_msgs_generate_messages_py)
endif()
