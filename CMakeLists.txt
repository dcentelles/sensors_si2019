cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)


project(sensors_si2019 CXX C)
message(STATUS "Building ${PROJECT_NAME}...")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  roslaunch
  std_srvs
  tf
  message_generation
  sensor_msgs
  tf_conversions
  uwsim
  mavlink_ros
)

SET(MODULES_DIR "modules" )

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()


####### HANDLE DEPENDENCIES ######

SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
if(GIT_FOUND)
  SET(SUBPROJECT_PATH ${DEPS_DIR}/${subdir})
  SET(SUBPROJECT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_PATH})
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${SUBPROJECT_FULL_PATH}
    OUTPUT_VARIABLE _sub_build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  SET(SUBPROJECT_GIT_NAME ${subdir}_${_sub_build_version})
  message(STATUS "Subproject generated git name: " ${SUBPROJECT_GIT_NAME})
  if (TARGET ${subdir})
      message(WARNING "Subproject ${SUBPROJECT_GIT_NAME} (${subdir}) has already been built!")
  else()
      add_subdirectory(${SUBPROJECT_PATH})
  endif()
else()
  message(STATUS "GIT not found")
endif()
ENDFOREACH()

######## END HANDLE DEPENDENCIES ######


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

SET(SRCPATH src/)

SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${SRCPATH}lib/include
    ${${UMCI_GIT}_INCLUDE_DIRS}
    )


## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-fPIC -std=c++14)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)
#SET(${PROJECT_NAME}_CPP_FILES
#    ${SRCPATH}lib/src/pid.cpp
#)
SET(${PROJECT_NAME}_netsim_scripts_CPP_FILES
    ${SRCPATH}lib/src/HILNetSimTracing.cpp
)

#add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
add_library(${PROJECT_NAME}_netsim_scripts SHARED ${${PROJECT_NAME}_netsim_scripts_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_netsim_scripts ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_netsim_scripts
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        )

install(DIRECTORY "${SRCPATH}lib/include/${PROJECT_NAME}" DESTINATION "include")


catkin_package(
   LIBRARIES
    ${PROJECT_NAME}_netsim_scripts
    ${${UMCI_GIT}_LIBRARIES}
   INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRS}
   CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    message_runtime
    merbots_whrov_msgs
    image_utils_ros_msgs
    sensor_msgs
    sensor_msgs
    tf_conversions
    mavlink_ros
    uwsim
)

roslaunch_add_file_check(launch)

message(STATUS "End ${PROJECT_NAME}")
