cmake_minimum_required(VERSION 2.8.3)
project(nostop_heading_calibration)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  std_msgs 
  base_math
  nav_msgs
)



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp rospy base_math wiringPi
)



###########
## Build ##
###########
include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.
include_directories(${OpenCV_INCLUDE_DIRS})


set(heading_calibration_SOURCES
        src/heading_calibration_node.cpp
        src/Heading_calibration.cpp
)


add_executable(heading_calibration_node ${heading_calibration_SOURCES})
add_dependencies(heading_calibration_node nostop_heading_calibration_gencpp)
target_link_libraries(heading_calibration_node ${catkin_LIBRARIES} base_math wiringPi)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS heading_calibration_node
 EXPORT nostop_heading_calibration
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 