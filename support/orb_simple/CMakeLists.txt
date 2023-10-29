cmake_minimum_required(VERSION 2.8)
project( FT )
find_package( OpenCV REQUIRED )

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
endif()		

include_directories(
    ThirdParty/DLib
    ${OpenCV_INCLUDE_DIRS}
)

add_library(FileFunctions Thirdparty/DLib/FileFunctions.cpp)
target_link_libraries(FileFunctions
    ${OpenCV_LIBS}
)

add_executable( orb_simple orb_simple.cpp)
target_link_libraries( orb_simple ${OpenCV_LIBS} FileFunctions)
