cmake_minimum_required(VERSION 3.0.2)
project(go2_mapping)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(go2_map_builder_node src/map_builder.cpp)
target_link_libraries(go2_map_builder_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(go2_map_exporter_node src/occupancy_map_exporter.cpp)
target_link_libraries(go2_map_exporter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS go2_map_builder_node go2_map_exporter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
