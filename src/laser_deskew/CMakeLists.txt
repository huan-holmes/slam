cmake_minimum_required(VERSION 3.1)
project(laser_deskew)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++11 support: OK")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Add new CMake Modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(USE_STACK_TRACE_LOGGER ON CACHE BOOL "Enable automatic stack trace logger of google-glog" FORCE)
if (USE_STACK_TRACE_LOGGER)
  message(STATUS "Stack trace logger: ENABLED")
  # Glog
  find_package(Glog REQUIRED)
  add_definitions(-DUSE_STACK_TRACE_LOGGER)
  message(STATUS "GLOG_INCLUDE_DIR: ${GLOG_INCLUDE_DIR}")
  message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")
else ()
  message(STATUS "Stack trace logger: DISABLED")
endif ()

# pcl
find_package(PCL REQUIRED
    COMPONENTS
    common
    )

message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_conversions
    )
#message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
#message(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")

# 设置动态生成位置
set(SHARED_LIB_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/lib)
message(STATUS "SHARED_LIB_PATH: ${SHARED_LIB_PATH}")

# 编译
catkin_package(
)

# 程序
add_executable(${PROJECT_NAME}_node
    src/deskew_node.cpp
    )

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIR}
    include
    )

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    )


install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(
    TARGETS ${PROJECT_NAME}_node
    # EXPORT  ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)