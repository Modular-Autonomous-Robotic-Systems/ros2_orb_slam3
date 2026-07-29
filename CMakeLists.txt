cmake_minimum_required(VERSION 3.8)
project(slam)

set(SLAM_TYPE "BASALT" CACHE STRING "Type of SLAM system to build (ORBSLAM3 or BASALT)")
set_property(CACHE SLAM_TYPE PROPERTY STRINGS "ORBSLAM3" "BASALT")

# Make sure to set this path before building the  
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Must use C++17 to make it compatible with rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(STATUS "Using flag -std=c++17.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # REDUNDANT?

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(your_custom_msg_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Yaml-cpp REQUIRED)
# find_package(MORB_SLAM REQUIRED COMPONENTS MORB_SLAM DBoW2 g2o sophus)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Header file locations [C++ node]
include_directories(
	${ament_INCLUDE_DIRS}
	$<INSTALL_INTERFACE:include>
	include 
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${OCTOMAP_INCLUDE_DIRS})



set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  visualization_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  # your_custom_msg_interface
  cv_bridge
  image_transport
  # OpenCV
  # Eigen3
  # Pangolin
  custom_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  rclcpp_lifecycle
  lifecycle_msgs
)

#* ORB SLAM3 as a shared library
# Whenever you add a new .h, .hpp or .cc, .cpp file in the ros2_orb_slam3/orb_slam3/include and os2_orb_slam3/orb_slam3/src directories, make sure to add them here as shown below

if(SLAM_TYPE STREQUAL "ORBSLAM3")
  add_subdirectory(orb_slam3)

  # Installed targets handle DBoW2 and g2o now

  # Add executable
  add_executable(orbslam3_mono_node
  	src/node.cpp
  	src/slam.cpp
  	src/orbslam3/slam.cpp
  	src/orbslam3/monocular.cpp
  	src/orbslam3/monocular_driver.cpp
  )
  set(USE_ORBLAM3 1)
  target_compile_definitions(orbslam3_mono_node PRIVATE USE_ORBSLAM3=${USE_ORBLAM3})
  # target_include_directories for orbslam3_mono_node are inherited from orb_slam3_lib
  # add_executable(morbslam_mono_node
  # 	src/node.cpp
  # 	src/slam.cpp
  # 	src/orbslam3/slam.cpp
  # 	src/morbslam/monocular.cpp
  # 	src/orbslam3/monocular_driver.cpp
  # )
  # target_include_directories()
  set(USE_MORBLAM 1)
  # target_compile_definitions(morbslam_mono_node PRIVATE USE_MORBSLAM=${USE_MORBSLAM})

  ament_target_dependencies(orbslam3_mono_node
    PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
  # ament_target_dependencies(morbslam_mono_node
  #   PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
  # )

  target_link_libraries(orbslam3_mono_node 
  	PUBLIC orb_slam3_lib
    	PUBLIC ${YAMLCPP_LIBRARY}
  	PUBLIC yaml-cpp) # Link a node with the internal shared library
  # target_link_libraries(morbslam_mono_node
  # 	PUBLIC MORB_SLAM::MORB_SLAM
  # 	PUBLIC MORB_SLAM::DBoW2
  # 	PUBLIC MORB_SLAM::g2o
  # 	PUBLIC MORB_SLAM::sophus
  #   	PUBLIC ${YAMLCPP_LIBRARY}
  # 	PUBLIC yaml-cpp
  # 	PUBLIC ${OCTOMAP_LIBRARIES})

  # Install our node and library
  install(TARGETS orbslam3_mono_node orb_slam3_lib DBoW2 g2o
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
  )

  # Help downstream packages to find transitive dependencies
  ament_export_dependencies(
    orb_slam3_lib
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
elseif(SLAM_TYPE STREQUAL "BASALT")
  # This is set to ensure that -march differences do not lead to object layout
  # differences for Eigen. Explcitly setting EIGEN_MAX_ALIGN_BYTES and -march
  # is defensive programming here
  add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16 NRT_EIGEN_ABI_PIN=16)

  add_subdirectory(ext/basalt)

  add_executable(basalt_slam_node
    src/node.cpp
    src/slam.cpp
    src/basalt/slam.cpp
    src/basalt/node.cpp
    src/basalt/driver.cpp
  )

  ament_target_dependencies(basalt_slam_node
    PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  target_link_libraries(basalt_slam_node
    PUBLIC basalt
    PUBLIC ${YAMLCPP_LIBRARY}
    PUBLIC yaml-cpp
  )

  install(TARGETS basalt_slam_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
  )

  ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
endif()

# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)


# Python node
# Install Python modules, submodules
ament_python_install_package(${PROJECT_NAME}) # Install the modules in ros2_orb_slam3/ros2_orb_slam3 folder

ament_package()
