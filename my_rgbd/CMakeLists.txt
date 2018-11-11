cmake_minimum_required(VERSION 3.1.2)
project(my_rgbd)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER "g++")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions( ${PCL_DEFINITIONS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

#add_executable(test src/main.cpp)
add_executable(my_rgbd  src/generatePointCloud.cpp)
target_link_libraries(my_rgbd ${OpenCV_LIBS} ${PCL_LIBRARIES})
