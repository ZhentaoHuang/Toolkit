# cmake needs this line
cmake_minimum_required(VERSION 3.5.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(server)

###
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    SET(PLATFORM_COMPILE_FLAGS "-mfpu=neon")
ELSE()
    SET(PLATFORM_COMPILE_FLAGS "-march=native")

    # some places of the algorithm are designed for 128 SIMD
    # so 128 SSE may slightly faster than 256 AVX, you may want this
#    SET(PLATFORM_COMPILE_FLAGS "-msse -msse2 -msse3 -msse4 -mssse3")  # SSE only
ENDIF()

# SET(PLATFORM_COMPILE_FLAGS "-DMIPP_NO_INTRINSICS")  # close SIMD
SET(COMMON_COMPILE_FLAGS "-fopenmp -Wall -Wno-sign-compare")
SET(CMAKE_CXX_FLAGS "${PLATFORM_COMPILE_FLAGS} ${COMMON_COMPILE_FLAGS} $ENV{CXXFLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
###

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "     Project: PROJECTNAME")
message(STATUS"       libaray_path: ${OpenCV_LIBS_DIRS}")



include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(
#  "include"  "${CMAKE_CURRENT_SOURCE_DIR}/MIPP/"
#)
include_directories (${INCLUDE_DIRECTORIES}  "include" "${CMAKE_CURRENT_SOURCE_DIR}/MIPP/")
set(SOURCE src/line2Dup.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (main main.cpp plane.cpp imgproc.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES}  ${CERES_LIBRARIES})

#add_executable (planar_segmentation main.cpp)
#target_link_libraries (planar_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES})
