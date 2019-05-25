cmake_minimum_required(VERSION 2.8) 
project( get_avoidance_vectors )
find_package( OpenCV REQUIRED )
find_package( realsense2 REQUIRED )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable( ${PROJECT_NAME} get_avoidance_vectors.cpp )

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY} )


