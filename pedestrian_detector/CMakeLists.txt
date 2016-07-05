file(GLOB detector_folder_source src/detector/*.cpp)
file(GLOB detector_folder_header include/detector/*.hpp)
file(GLOB tracker_lib_folder_source src/tracker/tracker_lib/*.cpp)
file(GLOB tracker_lib_folder_header include/tracker/tracker_lib/*.hpp)
file(GLOB tracker_folder_source src/tracker/*.cpp)
file(GLOB common_folder_source src/*.cpp)
file(GLOB tracker_folder_header include/tracker/*.hpp)
file(GLOB follower_folder_source src/follower/*.cpp)
file(GLOB follower_folder_header include/follower/*.hpp)

cmake_minimum_required(VERSION 2.8.3)
project(pedestrian_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  tf_conversions
  message_generation
  interactive_markers
  nav_msgs
)

find_package(OpenCV REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../binAndDataFiles)

add_message_files(
  FILES
  BoundingBox.msg
  Features.msg
  DetectionList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pedestrian_detector
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########
  SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall") #the -g option is implicit...
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -finline-functions -fpredictive-commoning -fgcse-after-reload -ftree-slp-vectorize -ftree-loop-distribute-patterns -fipa-cp-clone -funswitch-loops -fvect-cost-model -ftree-partial-pre -g")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}") #Default build mode is release mode

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

source_group("Detector Source Files" FILES ${detector_folder_source})
source_group("Detector Header Files" FILES ${detector_folder_header})
source_group("Tracker Source Files" FILES ${tracker_folder_source})
source_group("Tracker Header Files" FILES ${tracker_folder_header})
source_group("Follower Source Files" FILES ${follower_folder_source})
source_group("Follower Header Files" FILES ${follower_folder_header})

add_executable(detector ${detector_folder_header} ${detector_folder_source} ${common_folder_source})
target_link_libraries(detector ${catkin_LIBRARIES})
add_dependencies(detector pedestrian_detector_generate_messages_cpp)

add_library(tracker_lib ${tracker_lib_folder_header} ${tracker_lib_folder_source} ${common_folder_source})
target_link_libraries(tracker_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(tracker_lib pedestrian_detector_generate_messages_cpp)

add_executable(tracker ${tracker_folder_header} ${tracker_folder_source} ${common_folder_source})
target_link_libraries(tracker tracker_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(tracker pedestrian_detector_generate_messages_cpp)

add_executable(follower ${follower_folder_header} ${follower_folder_source} ${common_folder_source})
target_link_libraries(follower ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(follower pedestrian_detector_generate_messages_cpp)

##########################################
##Copy needed files to the bin directory##
##I think this is no longer needed      ##
##########################################
#
#configure_file(src/configuration.xml
#    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/configuration.xml COPYONLY)
#
#configure_file(src/StrongClassifierTreeStep4.txt
#    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/StrongClassifierTreeStep4.txt COPYONLY)
#
##This is not used in ACF. I must clean to code... but for now let's copy it to avoid problems...
#configure_file(src/rectangles.dat
#    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/rectangles.dat COPYONLY)
