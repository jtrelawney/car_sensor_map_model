cmake_minimum_required(VERSION 3.13)
project(simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -Wall")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

add_library(libfileops STATIC
    ../../cone_detection/common_code/file_operations.cpp
)

add_library(libimageops STATIC
    ../../cone_detection/common_code/image_operations.cpp
)

add_library(libdisplaymanager STATIC
    ../../cone_detection/common_code/display_manager_class.cpp
)

add_library(libmap_class STATIC
    ./map_class.cpp
)

add_library(libcar_class STATIC
    ./car_class.cpp
)

add_library(libsensor_class STATIC
    ./sensor_class.cpp
)

add_library(libsupport STATIC
    ./support.cpp
)

#include_directories("../cone_detection/common_code")

add_library(libvisualize_map_class STATIC
    ./visualize_map_class.cpp
)

#add_library(libfeaturedetect STATIC
#    ../../common_code/feature_detection.cpp
#)


target_include_directories(libfileops PUBLIC ../cone_detection/common_code/ )
target_include_directories(libimageops PUBLIC ../../cone_detection/common_code/ )
target_include_directories(libdisplaymanager PUBLIC ../cone_detection/common_code/ )
#target_include_directories(libsupport PUBLIC ../common_code/ )
#target_include_directories(libfeaturedetect PUBLIC ../common_code/ )

target_include_directories(libvisualize_map_class PUBLIC ../cone_detection/common_code/ )
target_link_libraries(libvisualize_map_class libimageops)

add_executable(simulation.exe simulation.cpp)
target_link_libraries(simulation.exe libfileops libdisplaymanager libimageops libmap_class libcar_class libsensor_class libsupport libvisualize_map_class ${OpenCV_LIBS})

add_executable(sensor_test.exe sensor_test.cpp)
target_link_libraries(sensor_test.exe libfileops libdisplaymanager libimageops libmap_class libcar_class libsensor_class libsupport libvisualize_map_class ${OpenCV_LIBS})
