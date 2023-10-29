cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros_wrapper)

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
   #$ENV{HOME}/ORB_SLAM3/catkin_ws/src/orb_slam3_ros_wrapper
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package (catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   cv_bridge
   image_transport
   tf
   sensor_msgs
   message_generation
)

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})

find_package(Pangolin REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Point2f.msg
   KeyPoint.msg
   KeyPoints.msg
   GlobalDescriptor.msg
   Descriptor.msg
)

## Generate added messages with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs message_runtime
   LIBRARIES {PROJECT_NAME}
)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${ORB_SLAM3_DIR}/Thirdparty/Sophus
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${catkin_INCLUDE_DIRS}
   ${Pangolin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
   -lboost_system
)

# mono node
add_executable (${PROJECT_NAME}_mono
   src/mono_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono
   ${LIBS}
)

add_executable (${PROJECT_NAME}_mono_acc_filtersubscriber
   src/mono_acc_filtersubscriber.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_acc_filtersubscriber
   ${LIBS}
)
#add_executable (${PROJECT_NAME}_mono_acc_subscriber
#   src/mono_acc_subscriber.cc
#   src/common.cc
#)
#target_link_libraries(${PROJECT_NAME}_mono_acc_subscriber
#   ${LIBS}
#)

# mono-inertial node
add_executable (${PROJECT_NAME}_mono_inertial
   src/mono_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_inertial
   ${LIBS}
)

# stereo node
add_executable (${PROJECT_NAME}_stereo
   src/stereo_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo
   ${LIBS}
)

# stereo node
add_executable (${PROJECT_NAME}_stereo_2
   src/stereo_node_2.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_2
   ${LIBS}
)

# accelerated stereo node
add_executable (${PROJECT_NAME}_stereo_acc
   src/stereo_node_acc.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_acc
   ${LIBS}
   ${ZLIB_LIBRARIES}
)

add_executable (${PROJECT_NAME}_stereo_acc_filtersubscriber
   src/stereo_acc_filtersubscriber.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_acc_filtersubscriber
   ${LIBS}
)

# stereo-inertial node
add_executable (${PROJECT_NAME}_stereo_inertial
   src/stereo_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_inertial
   ${LIBS}
)

# RGB-D node
add_executable (${PROJECT_NAME}_rgbd
   src/rgbd_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd
   ${LIBS}
)
