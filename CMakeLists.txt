cmake_minimum_required(VERSION 2.8)
project( Rotation_Tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
LIST(APPEND CMAKE_MODULE_PATH ./cmake_modules)
# OpenCV 
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
# 
include_directories( 
 include
${OpenCV_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR}
)

add_executable( Rotation_Tracking ./example/demo_video.cpp include/gms_matcher.h src/Converter.cpp include/Header.h include/file_operation.hpp src/file_operation.cpp)
target_link_libraries( Rotation_Tracking ${OpenCV_LIBS} )
