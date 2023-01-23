cmake_minimum_required(VERSION 2.8)
project( FT )
find_package( OpenCV REQUIRED )

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
endif()		

add_executable( renamenodes renamenodes.cpp)
target_link_libraries( renamenodes ${OpenCV_LIBS} )
