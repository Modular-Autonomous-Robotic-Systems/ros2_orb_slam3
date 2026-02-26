cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(Pangolin REQUIRED)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

add_library(orb_slam3_lib SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/OctoMapBuilder.cpp
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
)

target_include_directories(
	orb_slam3_lib
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
    PUBLIC ${Pangolin_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
  -lboost_system
  -lboost_serialization
  -lcrypto
)
