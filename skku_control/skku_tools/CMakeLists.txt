cmake_minimum_required(VERSION 3.5)
project(skku_tools)
set(CMAKE_CXX_COMPILER g++)
add_compile_options(-std=c++17)
unset(_CATKIN_CURRENT_PACKAGE)

 # check if this is being built as a ROS package
if (CATKIN_DEVEL_PREFIX)
  set(CATKIN_BUILD 1)
endif()


set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



find_package(catkin REQUIRED COMPONENTS
  dsr_msgs
  controller_manager
  hardware_interface
  serial
  roscpp
  actionlib
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skku_tools
  CATKIN_DEPENDS 
    dsr_msgs
    controller_manager 
    hardware_interface 
    roscpp 
    serial
)

find_package(SFML COMPONENTS graphics window system)

SET( COMMON_INC_DIRECTORY
  ../../common/include
)
SET( COMMON_LIB_DIRECTORY
  ../../common/lib
)

SET( SERIAL_SRC_FILES
  ../../common/src/dsr_serial.cpp
)

add_library(skku_tools SHARED
  src/control_loop.cpp
  src/control_tools.cpp
  src/control_state.cpp
  src/duration.cpp
  src/impedance_controller.cpp
)

add_dependencies(skku_tools
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)



target_link_libraries(skku_tools
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES} # yaml-cpp 라이브러리 링크
  DRFL PocoFoundation PocoNet
  fltk X11 dl m Xext Xinerama Xft fontconfig Xrender pthread
  /home/rbl/libtensorflow-cpu-linux-x86_64-2.6.0/lib/libtensorflow.so
  
)


target_include_directories(skku_tools PUBLIC
  include  
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR} # yaml-cpp 헤더 파일 포함
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${COMMON_INC_DIRECTORY}
  ${CMAKE_CURRENT_SOURCE_DIR}/config
  /home/rbl/cppflow/include
  /home/rbl/tensorflow-2.6.0
  ${EIGEN3_INCLUDE_DIRS}

)


if($ENV{ROS_DISTRO} STREQUAL "kinetic")
  # check OS 32bit or 64bit ?  
  if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
  else()
    LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_32")
  endif()
elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
    message("Unknown ROS distro:")
    message($ENV{ROS_DISTRO})
endif()

# install

install(TARGETS skku_tools
    EXPORT skkuTargets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


