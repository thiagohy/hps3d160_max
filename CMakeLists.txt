cmake_minimum_required(VERSION 3.16)
project(hps3d160_max VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(HPS3D160_MAX_WITH_PCL "Enable PCL point cloud support" ON)
option(HPS3D160_MAX_WITH_ROS "Enable ROS integration" ON)

# --- SDK paths ---
set(HPS_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hps_sdk)
set(HPS_SDK_INCLUDE ${HPS_SDK_ROOT}/include)
set(HPS_SDK_LIB ${HPS_SDK_ROOT}/lib)

# --- Optional dependencies ---
find_package(PCL 1.8 QUIET)
if(PCL_FOUND AND HPS3D160_MAX_WITH_PCL)
    set(HPS_HAVE_PCL TRUE)
    message(STATUS "PCL found: enabling point cloud PCD/PLY support")
else()
    set(HPS_HAVE_PCL FALSE)
    message(STATUS "PCL not found: disabling PCD/PLY support")
endif()

set(HPS_HAVE_ROS FALSE)
if(HPS3D160_MAX_WITH_ROS)
    find_package(catkin QUIET COMPONENTS roscpp sensor_msgs pcl_conversions pcl_ros std_msgs)
    if(catkin_FOUND)
        set(HPS_HAVE_ROS TRUE)
        message(STATUS "catkin found: enabling ROS node")
    else()
        message(STATUS "catkin not found: building standalone tool only")
    endif()
endif()

# --- SDK library ---
# Recover the internal handle-array layout from the shipped binary so the
# advanced (HE201_Api*) entry points can be used safely.
set(HPS_SDK_SO ${HPS_SDK_LIB}/libHPS3DSDK64_1-8-6.so)
if(EXISTS ${HPS_SDK_SO} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_sdk_layout.sh)
    execute_process(
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_sdk_layout.sh
                ${HPS_SDK_SO}
                ${CMAKE_CURRENT_SOURCE_DIR}/include/hps3d160_max/generated/sdk_layout.h
        RESULT_VARIABLE HPS_LAYOUT_RESULT
        OUTPUT_VARIABLE HPS_LAYOUT_OUTPUT
        ERROR_VARIABLE HPS_LAYOUT_ERROR
    )
    if(HPS_LAYOUT_RESULT EQUAL 0)
        message(STATUS "SDK layout: ${HPS_LAYOUT_OUTPUT}")
    else()
        message(STATUS "SDK layout extraction skipped: ${HPS_LAYOUT_ERROR}")
    endif()
endif()

add_library(hps_sdk STATIC
    ${HPS_SDK_INCLUDE}/HPS3DUser_IF.c
)
target_include_directories(hps_sdk PUBLIC ${HPS_SDK_INCLUDE})
target_link_directories(hps_sdk PUBLIC ${HPS_SDK_LIB})
target_link_libraries(hps_sdk PUBLIC HPS3DSDK64_1-8-6)
set_target_properties(hps_sdk PROPERTIES
    OUTPUT_NAME hps_sdk
)

# --- Core library ---
file(GLOB HPS_CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/device_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/capture_service.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/filter_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pointcloud_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/recorder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/health_monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/multi_camera_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sdk_bridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/capability_probe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hdr_compare.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/throughput_bench.cpp
)
add_library(hps3d160_core STATIC ${HPS_CORE_SOURCES})
target_include_directories(hps3d160_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hps3d160_max
)
target_link_libraries(hps3d160_core PUBLIC hps_sdk ${CMAKE_DL_LIBS})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/hps3d160_max/generated/sdk_layout.h)
    target_compile_definitions(hps3d160_core PUBLIC HPS3D160_MAX_HAVE_SDK_LAYOUT=1)
endif()
if(HPS_HAVE_PCL)
    target_compile_definitions(hps3d160_core PUBLIC HPS3D160_MAX_WITH_PCL=1)
    target_include_directories(hps3d160_core PUBLIC ${PCL_INCLUDE_DIRS})
    target_link_libraries(hps3d160_core PUBLIC ${PCL_LIBRARIES})
endif()

# --- ROS publisher (optional) ---
if(HPS_HAVE_ROS)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hps3d160_core
        CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros std_msgs
    )
    add_library(hps_ros_publisher STATIC src/ros_publisher.cpp)
    target_include_directories(hps_ros_publisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
    target_compile_definitions(hps_ros_publisher PUBLIC HPS3D160_MAX_WITH_ROS=1)
    target_link_libraries(hps_ros_publisher PUBLIC hps3d160_core ${catkin_LIBRARIES})
    if(HPS_HAVE_PCL)
        target_compile_definitions(hps_ros_publisher PUBLIC HPS3D160_MAX_WITH_PCL=1)
    endif()
endif()

# --- Main executable ---
add_executable(hps3d160_max_node src/main.cpp)
target_include_directories(hps3d160_max_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/hps3d160_max)
target_link_libraries(hps3d160_max_node PRIVATE hps3d160_core)
if(HPS_HAVE_ROS)
    target_compile_definitions(hps3d160_max_node PRIVATE HPS3D160_MAX_WITH_ROS=1)
    target_include_directories(hps3d160_max_node PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(hps3d160_max_node PRIVATE hps_ros_publisher ${catkin_LIBRARIES})
endif()
if(HPS_HAVE_PCL)
    target_compile_definitions(hps3d160_max_node PRIVATE HPS3D160_MAX_WITH_PCL=1)
endif()

# RPATH for SDK
set_target_properties(hps3d160_max_node PROPERTIES
    BUILD_RPATH "${HPS_SDK_LIB}"
    INSTALL_RPATH "${HPS_SDK_LIB}"
)

install(TARGETS hps3d160_max_node RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)