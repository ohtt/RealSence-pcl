project(LRS_PCL)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt/5.5/gcc_64 /usr/local /usr/local/opt/ /usr/local/opt/pcl　)

find_package( PCL REQUIRED )
find_package( Qt5 REQUIRED COMPONENTS Widgets Core )
find_package(realsense2 REQUIRED)

include( CheckCXXCompilerFlag )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2" )
set(VTK_LEGACY_SILENT:BOOL ON)
include_directories( ${PCL_INCLUDE_DIRS} ${realsense_INCLUDE_DIR})
link_directories( ${PCL_LIBRARY_DIRS} ${realsense2_LIBRARY_DIR} )
add_definitions( ${PCL_DEFINITIONS} )

target_link_libraries( LRS_PCL ${PCL_LIBRARIES} ${realsense2_LIBRARY}  )