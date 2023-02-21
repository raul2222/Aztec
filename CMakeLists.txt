cmake_minimum_required(VERSION 3.9.5)
project(cs20_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()



FILE(GLOB CS20_SDK_SRC
 
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu20.04_x64/*.so"

)

set(req_deps
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
  "camera_info_manager"
  "image_transport"
  "cv_bridge"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package( OpenCV REQUIRED )

find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(
  include/libsynexens3
  src
  ${OpenCV_INCLUDE_DIRS}
  include
  lib/ubuntu20.04_x64
)



link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu20.04_x64)


#add_library(image_cs20 SHARED
#  ${CS20_SDK_SRC}
#)
#ament_target_dependencies(CS20_composition_node ${req_deps})

# build direct ROS 1 port

ament_auto_add_executable(image_cs20 src/image_cs.cpp)
target_link_libraries(image_cs20 ${CS20_SDK_SRC})
#ament_target_dependencies(image_cs20 ${req_deps}  ${CS20_SDK_SRC})

#ament_auto_add_executable(CS20NodeClient src/client.cpp)
#ament_target_dependencies(CS20NodeClient ${req_deps})

# build composition node
#ament_auto_add_executable(CS20_composition src/init_cs20_tof_lidar.cpp)
#target_link_libraries(CS20_composition CS20_composition_node)
#ament_target_dependencies(CS20_composition ${req_deps})

install(
  DIRECTORY lib/
  DESTINATION lib
)

#install(
#  DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/
#)

install(
  TARGETS image_cs20 
  DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY launch rviz
#  DESTINATION share/${PROJECT_NAME}
#)

# Install the shared library composition node
#install(
#  TARGETS cs20_composition_node
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# TODO(hallen): port this
# install(DIRECTORY sdk
#   USE_SOURCE_PERMISSIONS
# )

ament_auto_package()