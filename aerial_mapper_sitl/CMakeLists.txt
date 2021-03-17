cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_sitl)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

cs_add_executable(${PROJECT_NAME}
  src/simulation_node.cc
  src/simulation.cc
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
  
##########
# EXPORT #
##########
cs_install()
cs_export()
