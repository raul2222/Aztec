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
 
  "${CS20_SDK_PATH}/lib/ubuntu20.04_x64/*.so"
  "${CS20_SDK_PATH}/lib/*.so"
)

set(req_deps
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package( OpenCV REQUIRED )



ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(
  #include
  src
  ${OpenCV_INCLUDE_DIRS}
  include
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu20.04_x64)

# build composition node
#add_library(CS20_composition_node SHARED
#  src/cs20_node.cpp
#  ${CS20_SDK_SRC}
#)
#ament_target_dependencies(CS20_composition_node ${req_deps})

# build direct ROS 1 port

ament_auto_add_executable(cs20node src/cs20node.cpp ${CS20_SDK_SRC})
#target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
#ament_target_dependencies(${PROJECT_NAME} ${req_deps})

#ament_auto_add_executable(CS20NodeClient src/client.cpp)
#ament_target_dependencies(CS20NodeClient ${req_deps})

# build composition node
#ament_auto_add_executable(CS20_composition src/init_cs20_tof_lidar.cpp)
#target_link_libraries(CS20_composition CS20_composition_node)
#ament_target_dependencies(CS20_composition ${req_deps})

install(
  TARGETS cs20node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

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