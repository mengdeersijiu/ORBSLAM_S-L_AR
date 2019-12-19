cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Debug)
ENDIF()

# Add -O0 to remove optimizations when using gcc
IF(CMAKE_COMPILER_IS_GNUCC)
   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
   set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(aruco_DIR /home/mxy/Downloads/aruco-3.1.2/build)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(CUDA REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(aruco REQUIRED)
find_package(mynteye QUIET)
find_package(mynteyed QUIET)
if(mynteye_FOUND)
   message(STATUS "MYNT-EYE-S-SDK found.")
else()
   message(STATUS "MYNT-EYE-S-SDK not found.")
endif()

if(mynteyed_FOUND)
   message(STATUS "MYNT-EYE-D-SDK found.")
else()
   message(STATUS "MYNT-EYE-D-SDK not found.")
endif()


find_package( OpenGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIR})
find_package( GLUT REQUIRED )
include_directories(${GLUT_INCLUDE_DIR})
find_package( GLEW REQUIRED )
include_directories(${GLEW_INCLUDE_DIR})
find_package( PNG REQUIRED )
include_directories(${PNG_INCLUDE_DIR})
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../../cmake_modules" ${CMAKE_MODULE_PATH})
find_package(GLFW REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})



include_directories(
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
        ${PROJECT_SOURCE_DIR}/src/openglAR/include
${Pangolin_INCLUDE_DIRS}
${aruco_INCLUDE_DIRS}
${aruco_INCLUDE_DIRS}/include
)

message(STATUS "Compile With save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
   message(FATAL_ERROR "Can't find libboost_serialization")
endif()

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${BOOST_SERIALIZATION}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${PROJECT_SOURCE_DIR}/../../../lib/libboost_filesystem.so
${PROJECT_SOURCE_DIR}/../../../lib/libboost_system.so
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
rosbuild_add_executable(MonoAR
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
src/AR/keyboard.h
src/AR/keyboard.cc
)

target_link_libraries(MonoAR
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# Node for mynteye stereo camera
rosbuild_add_executable(ros_mynteye_stereo
src/ros_mynteye_stereo.cc
)

target_link_libraries(ros_mynteye_stereo
${LIBS}
)

# Node for Stereo camera (Augmented Reality Demo)
rosbuild_add_executable(StereoAR
src/AR/ros_stereo_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
src/AR/keyboard.h
src/AR/keyboard.cc
)

target_link_libraries(StereoAR
${LIBS}
)

message("GLOG_LIBRARIES: ${GLOG_LIBRARIES}")
add_executable(stereo_myntAR
src/AR/stereo_myntAR.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
)
target_link_libraries(stereo_myntAR  ${LIBS} ${GLOG_LIBRARIES} mynteye)

SET(SRC_FOLDER ${PROJECT_SOURCE_DIR}/src/openglAR)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

ADD_EXECUTABLE(stereo_openglAR ${SRC_FILES})

TARGET_LINK_LIBRARIES(stereo_openglAR
        #${ZED_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
        #ygx
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLEW_LIBRARIES}
        /usr/lib/libglfw.so.3
        ${PNG_LIBRARY}
        #ygx
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${aruco_LIBS}
        ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
        mynteye)
