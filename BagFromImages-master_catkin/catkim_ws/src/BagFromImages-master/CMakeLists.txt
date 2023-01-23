set(CMAKE_EXPORT_COMPILE_COMMANDS 1 )
set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W -Wno-unused-parameter")

cmake_minimum_required(VERSION 2.4.6)
project(BagFromImages)
#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rosbag
    cv_bridge
    image_transport
    OpenCV
    sensor_msgs
    message_generation
)

FIND_PACKAGE(ZLIB REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)
find_package(PkgConfig)

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

#catkin_package()
catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs message_runtime
   LIBRARIES {PROJECT_NAME}
)

include_directories(
    ThirdParty/DLib
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(FileFunctions Thirdparty/DLib/FileFunctions.cpp)
target_link_libraries(FileFunctions
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(main main.cc)
target_link_libraries(main
    FileFunctions
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZLIB_LIBRARIES}
)
