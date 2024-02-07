cmake_minimum_required(VERSION 3.0.2)
project(mpc_low)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DACADOS_SILENT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DACADOS_SILENT")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES mpc_low
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


cs_add_library(mpc_low
  src/low_solver.cpp
  solver/Wheelchair_model/Wheelchair_expl_ode_fun.c
  solver/Wheelchair_model/Wheelchair_expl_vde_forw.c
  solver/Wheelchair_model/Wheelchair_expl_vde_adj.c
  solver/Wheelchair_constraints/Wheelchair_constr_h_fun_jac_uxt_zt.c
  solver/Wheelchair_constraints/Wheelchair_constr_h_fun.c
  solver/acados_solver_Wheelchair.c
  solver/acados_sim_solver_Wheelchair.c
  solver/acados_solver_holder.cpp
  )

target_include_directories(mpc_low PUBLIC
	solver/
	solver/acados_libs/include
	solver/acados_libs/include/acados
	solver/acados_libs/include/acados_c
	solver/acados_libs/include/blasfeo/include
	solver/acados_libs/include/hpipm/include
	)

add_executable(custom_odometry_node_real src/custom_odometry_real.cpp)
target_link_libraries(custom_odometry_node_real ${catkin_LIBRARIES})

add_executable(custom_odometry_node_sim src/custom_odometry_sim.cpp)
target_link_libraries(custom_odometry_node_sim ${catkin_LIBRARIES})

add_executable(mpc_low_node src/low_solver.cpp)
target_link_libraries(mpc_low_node ${catkin_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/solver/acados_libs/lib/libacados.so
        ${CMAKE_CURRENT_SOURCE_DIR}/solver/acados_libs/lib/libhpipm.so
        ${CMAKE_CURRENT_SOURCE_DIR}/solver/acados_libs/lib/libblasfeo.so
        mpc_low)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/solver/acados_libs/lib DESTINATION ${CATKIN_DEVEL_PREFIX}) 


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gen3_mpc_joint.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
cs_install()
cs_export()
