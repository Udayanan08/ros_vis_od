cmake_minimum_required(VERSION 3.12)

project(visual_odometry	 VERSION 0.0.1
			  DESCRIPTION "HELLO"
			  LANGUAGES CXX)

message(${CMAKE_CXX_COMPILER_ID})
message(${CMAKE_CXX_COMPILER_VERSION})

set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)

# set(dependencies 
# 	rclcpp
# 	yaml-cpp
# 	std_msgs
# 	sensor_msgs
# 	cv_bridge
# 	image_transport
# 	OpenCV)


add_library(feature src/feature.cpp)
target_link_libraries(feature ${OpenCV_LIBS} yaml-cpp )
# target_link_libraries(feature ${OpenCV_LIBS})

add_executable(vis_od src/vis_od.cpp)
ament_target_dependencies(vis_od yaml-cpp OpenCV rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(vis_od feature)


# target_include_directories(vis_od
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# 	${OpenCV_INCLUDE_DIRS})



# target_include_directories(run PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/config)
# ${Pangolin_LIBRARIES}


# ament_export_include_directories(include)
# ament_export_dependencies(dependencies)

install(
	DIRECTORY config/
	DESTINATION lib/config
)
install(
  DIRECTORY include/
  DESTINATION lib/include
)
install(
  DIRECTORY launch/
  DESTINATION lib/launch
)

install(
  TARGETS vis_od DESTINATION lib/${PROJECT_NAME})

# install(
#   EXPORT my_libraryTargets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )
ament_package()


#include_directories("/home/drdo/cpp/libraries/eigen/install")
# include_directories(${PROJECT_SOURCE_DIR/include})

# target_link_libraries(feature ${OpenCV_LIBS})

# add_executable(run src/vis_od.cpp)

# install(DIRECTORY include/ DESTINATION include/)
# install(TARGETS run DESTINATION lib/${PROJECT_NAME})

# ament_target_dependencies(run ${OpenCV_LIBS} yaml-cpp)

# ament_package()
# # target_include_directories(run PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/config)
# # target_link_libraries(run feature ${OpenCV_LIBS} yaml-cpp)# ${Pangolin_LIBRARIES}
