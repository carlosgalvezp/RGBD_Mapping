cmake_minimum_required(VERSION 2.8)

project(rgbdtools)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-modules/")

include_directories(${PROJECT_SOURCE_DIR}/include)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Boost:
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Dependencies - OpenCV:
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - PCL:
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Dependencies - Suitesparse:
FIND_PACKAGE(SuiteSparse REQUIRED)
include_directories(${SUITESPARSE_INCLUDE_DIRS})

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually 
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

####################################################
# Build rgbdtools library.
file(GLOB_RECURSE DIR_SOURCES "src/*.cpp")
file(GLOB_RECURSE DIR_HEADERS "include/*.h")


add_library (rgbdtools SHARED
  ${DIR_SOURCES}
  ${DIR_HEADERS})

target_link_libraries (rgbdtools
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES})
  
install(TARGETS rgbdtools DESTINATION lib)
install(DIRECTORY include/rgbdtools DESTINATION include)

####################################################
# Build global cloud align applications.

add_executable(global_cloud_align apps/global_cloud_align.cpp)
target_link_libraries (global_cloud_align rgbdtools) 
install(TARGETS global_cloud_align DESTINATION bin)

####################################################
# Build keyframe associations applications.

add_executable(keyframe_associations apps/keyframe_associations.cpp)
target_link_libraries (keyframe_associations rgbdtools)
install(TARGETS keyframe_associations DESTINATION bin)

