cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(moss_rock VERSION 1.0 LANGUAGES CXX)

# IF(NOT CMAKE_BUILD_TYPE)
#   SET(CMAKE_BUILD_TYPE Release)
# ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Set compiler standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7) # required for eigen
    message(FATAL_ERROR "Insufficient gcc version must be at least 7")
  endif()
endif()

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.2 not found.")
   endif()

MESSAGE(STATUS "OPENCV VERSION:")
MESSAGE(STATUS ${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# include_directories(
# ${PROJECT_SOURCE_DIR}
# ${PROJECT_SOURCE_DIR}/include
# ${PROJECT_SOURCE_DIR}/include/CameraModels
# ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
# ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2
# ${EIGEN3_INCLUDE_DIR}
# ${Pangolin_INCLUDE_DIRS}
# )

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} STATIC
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
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
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)


add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

target_include_directories(${PROJECT_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o>
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.a
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.a
-lboost_serialization
-lcrypto
)

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)

add_executable(mono_test
     ros/mono_ros_node.cc
)
# Link the library to the executable
target_link_libraries(mono_test ${PROJECT_NAME})
ament_target_dependencies(mono_test
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "foxglove_msgs"
)

install(TARGETS
mono_test
  DESTINATION lib/${PROJECT_NAME})

ament_package() 