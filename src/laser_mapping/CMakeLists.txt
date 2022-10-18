cmake_minimum_required(VERSION 3.1)
project(laser_mapping)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++11 support: OK")

set(CMAKE_BUILD_TYPE "Release")

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


find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
find_package(Threads REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    nav_msgs
    sensor_msgs
    tf
    common_msg
    rosbag_storage
)

catkin_package(
#   INCLUDE_DIRS include 3rdparty/laser_mapping_node/include
  LIBRARIES david_karto sba spa_solver tools
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs common_msg tf
#   DEPENDS david_karto sba spa_solver
)

# 设置动态生成位置
set(SHARED_LIB_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/lib)
message(STATUS "SHARED_LIB_PATH: ${SHARED_LIB_PATH}")

# 第三方头文件库位置
set(THIRD_PARTY_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../3rdparty/include")
message(STATUS "THIRD_PARTY_INCLUDE_DIRS: ${THIRD_PARTY_INCLUDE_DIRS}")

# 加载其他模块
add_subdirectory(src/david_karto)
add_subdirectory(src/sba)
add_subdirectory(src/spa_solver)
add_subdirectory(src/tools)

# 编译

find_package(Boost REQUIRED
    COMPONENTS
    thread
    )

add_executable(${PROJECT_NAME}_node
    src/laser_mapping.cpp
    src/slam_manager.cpp
    )

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${THIRD_PARTY_INCLUDE_DIRS}
    include
    ${GLOG_INCLUDE_DIR}
    )

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    Threads::Threads
    spa_solver
    tools
    )



# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   config
   launch
   rviz
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
