# cmake needs this line
cmake_minimum_required(VERSION 3.5.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(server)

if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

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
SET(-DGLIBCXX_USE_CXX11_ABI=ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")


# Set OS-specific things here
if(WIN32)
    if (ENABLE_GUI)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE")
        # Filament is built with /MT[d], but by default CMake automatically
        # sets "/MD" and "/MDd". They can't coexist, so we need to replace them.
        string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
        string(REPLACE "/MDd" "/MTd" CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
        string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
        string(REPLACE "/MDd" "/MTd" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
    endif()
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
    add_definitions(-DUNIX)
    add_compile_options(-Wno-deprecated-declarations)
    add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)
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

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#find_package( Open3D  REQUIRED)

 
#include_directories(${Open3D_INCLUDE_DIRS})
#link_directories(${Open3D_LIBRARY_DIRS})

if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "open3d_include -> ${Open3D_INCLUDE_DIRS}")
    message(STATUS "open3d_libs -> ${Open3D_LIBRARIES}")

    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})
    include_directories(${Open3D_INCLUDE_DIRS})


    #add_executable(camera_pose_trajectory main.cpp)
    #target_link_libraries(camera_pose_trajectory ${Open3D_LIBRARIES})
    #target_include_directories(camera_pose_trajectory PUBLIC ${Open3D_INCLUDE_DIRS})

    # Hot fix windows dll not found issue, assumming we're using the Release build
    option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)
    if(WIN32 AND BUILD_SHARED_LIBS)
        message("Will copy Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET camera_pose_trajectory POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()
    if(WIN32 AND ENABLE_GUI)
        message("Will copy SDL2.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET camera_pose_trajectory POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/lib/SDL2.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()

else ()
    message(SEND_ERROR "Open3D not found")
endif ()
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (main main.cpp imgproc.cpp threedproc.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES}  ${CERES_LIBRARIES} ${Open3D_LIBRARIES})
