cmake_minimum_required( VERSION 3.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
if (WIN32)
    find_package(Eigen3 CONFIG REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIRS})
else ()
    include_directories( "/usr/include/eigen3" )
endif ()

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
