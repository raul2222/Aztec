cmake_minimum_required(VERSION 3.4.1)
project("examples")

set(TARGET_NAME libsynexens3_demo)


message("configure ${TARGET_NAME}")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/ubuntu20.04_x64)


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


set(LIBSYNEXENS3 synexens3)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fopenmp")

add_executable(depth depth.cpp)
target_link_libraries(depth  PRIVATE ${LIBSYNEXENS3}  ${OpenCV_LIBS} )
set_property(TARGET depth PROPERTY FOLDER examples)


