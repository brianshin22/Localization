# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localize: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilocalize:/home/mpc/Localization/workspace/src/localize/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localize_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg" NAME_WE)
add_custom_target(_localize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localize" "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(localize
  "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localize
)

### Generating Services

### Generating Module File
_generate_module_cpp(localize
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localize
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localize_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localize_generate_messages localize_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg" NAME_WE)
add_dependencies(localize_generate_messages_cpp _localize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localize_gencpp)
add_dependencies(localize_gencpp localize_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localize_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(localize
  "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localize
)

### Generating Services

### Generating Module File
_generate_module_lisp(localize
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localize
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localize_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localize_generate_messages localize_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg" NAME_WE)
add_dependencies(localize_generate_messages_lisp _localize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localize_genlisp)
add_dependencies(localize_genlisp localize_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localize_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(localize
  "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localize
)

### Generating Services

### Generating Module File
_generate_module_py(localize
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localize
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localize_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localize_generate_messages localize_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Localization/workspace/src/localize/msg/LaneMeasure.msg" NAME_WE)
add_dependencies(localize_generate_messages_py _localize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localize_genpy)
add_dependencies(localize_genpy localize_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localize_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localize
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(localize_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localize
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(localize_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localize)
  install(CODE "execute_process(COMMAND \"/home/mpc/.virtualenvs/research/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localize\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localize
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(localize_generate_messages_py std_msgs_generate_messages_py)
