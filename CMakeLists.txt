project(mavsdk_ros)
cmake_minimum_required(VERSION 3.5.1)

#####################
## Project Options ##
#####################

## Force some variables that could be defined in the command line to be written to cache
option(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                  OFF)
option(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics listed in C standard"  ON)
option(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers"  OFF)

add_compile_options(-std=c++17)
add_compile_options(-Wall)

if(${WARNINGS_ANSI_ISO})
   add_compile_options(-Wcast-align)
   add_compile_options(-Wpedantic)
   add_compile_options(-Wextra)
else()
   add_compile_options(-Wno-narrowing)
   add_compile_options(-Wno-delete-non-virtual-dtor)
   add_compile_options(-Wno-unnamed-type-template-args)
endif()

if(${WARNINGS_ARE_ERRORS})
   add_compile_options(-Werror)
endif()

if(${WARNINGS_EFFCPP})
   add_compile_options(-Weffc++)
endif()


find_package(MAVSDK REQUIRED)
set(MAVSDK_COMPONENTS 
"MAVSDK::mavsdk"
"MAVSDK::mavsdk_telemetry_robotic_vehicle"
"MAVSDK::mavsdk_alarm_robotic_vehicle"
"MAVSDK::mavsdk_checklist_robotic_vehicle"
"MAVSDK::mavsdk_command_robotic_vehicle"
"MAVSDK::mavsdk_hl_action_robotic_vehicle"
"MAVSDK::mavsdk_inspection_robotic_vehicle")

##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED 
   COMPONENTS  rospy roscpp std_msgs sensor_msgs)

#catkin_python_setup()

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## Build a C++ library
file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_SRCS "src/*.c*")
file(GLOB_RECURSE SRC_HDRS "src/*.h*")

add_library(${PROJECT_NAME}            ${SRC_SRCS} ${SRC_HDRS} ${SRC_INCS})
add_dependencies(${PROJECT_NAME}       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${MAVSDK_COMPONENTS})

add_executable(${PROJECT_NAME}_node          src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node   ${PROJECT_NAME})

#########################
## Print configuration ##
#########################

message(STATUS)
message(STATUS "------------ General configuration for - ${PROJECT_NAME} ------------")
message(STATUS)
message(STATUS "WARNINGS_ANSI_ISO   = ${WARNINGS_ANSI_ISO}")
message(STATUS "WARNINGS_ARE_ERRORS = ${WARNINGS_ARE_ERRORS}")
message(STATUS "WARNINGS_EFFCPP     = ${WARNINGS_EFFCPP}")
message(STATUS)
message(STATUS "BUILD_SHARED_LIBS   = ${BUILD_SHARED_LIBS}")
message(STATUS)
message(STATUS "-------------------------- Dependencies --------------------------")
message(STATUS)
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)
