cmake_minimum_required(VERSION 3.5)
project(skku_control)

# 컴파일 옵션 설정
add_compile_options(-std=c++17)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 패키지 찾기
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dsr_msgs
  controller_manager
  hardware_interface
  serial
  roscpp
  actionlib
 
)

# 헤더 파일 포함 경로 설정
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 스레드 사용 설정
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# catkin 패키지 옵션 설정
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dsr_msgs controller_manager hardware_interface roscpp serial
)

# 공통 파일 설정
set(COMMON_INC_FILES
  ../common/include
)
set(COMMON_LIB_FILES
  ../common/lib
)
set(SERIAL_SRC_FILES
  ../common/src/dsr_serial.cpp
)
set(SRC_FILES
  ${SERIAL_SRC_FILES}
  src/dsr_hw_interface.cpp
  src/SKKU_control_node.cpp
)

# ROS 배포판에 따른 링크 디렉토리 설정
if($ENV{ROS_DISTRO} STREQUAL "kinetic")
  if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    link_directories("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
  else()
    link_directories("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_32")
  endif()
elseif($ENV{ROS_DISTRO} STREQUAL "melodic" OR $ENV{ROS_DISTRO} STREQUAL "noetic")
  link_directories("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
  message("Unknown ROS distro: $ENV{ROS_DISTRO}")
endif()

# 추가 디렉토리 설정
link_directories("${COMMON_LIB_FILES}/noetic/x86_64")

# 서브디렉토리 추가
add_subdirectory(skku_tools)

# 실행 파일 설정 및 라이브러리 연결
add_executable(skku_control_node ${SRC_FILES})
add_executable(publisher_node src/trajectory_publisher_node.cpp)
add_executable(can_communication_node src/can_communication_node.cpp)

# 각 타겟에 라이브러리 연결
target_link_libraries(skku_control_node ${catkin_LIBRARIES} DRFL PocoFoundation PocoNet skku_tools) 
target_link_libraries(publisher_node ${catkin_LIBRARIES})
target_link_libraries(can_communication_node ${catkin_LIBRARIES})

# 의존성 설정
add_dependencies(skku_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(can_communication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# include 디렉토리 설정
target_include_directories(skku_control_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(skku_control_node PUBLIC include ${COMMON_INC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(publisher_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(publisher_node PUBLIC include ${COMMON_INC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(can_communication_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(can_communication_node PUBLIC include ${COMMON_INC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 설치 설정
install(TARGETS skku_control_node can_communication_node publisher_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
