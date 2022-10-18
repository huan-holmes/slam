cmake_minimum_required(VERSION 3.1)
project(laser_localization)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++11 support: OK")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 调试相关
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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

if (CMAKE_COMPILER_IS_GNUCXX)
  # High level of warnings.
  # The -Wno-long-long is required in 64bit systems when including sytem headers.
  # The -Wno-variadic-macros was needed for Eigen3, StdVector.h
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  # Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
  if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif ()
endif ()

# 第三方头文件库位置
set(THIRD_PARTY_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../3rdparty/include")
message(STATUS "THIRD_PARTY_INCLUDE_DIRS: ${THIRD_PARTY_INCLUDE_DIRS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        std_msgs
        nav_msgs
        sensor_msgs
        mrpt_msgs
        common_msg
        tf
        )

# 设置动态生成位置
set(SHARED_LIB_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/lib/)
message(STATUS "SHARED_LIB_PATH: ${SHARED_LIB_PATH}")

find_package(MRPT 2.3 REQUIRED
    obs
    slam)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")

# 添加mrpt_bridge
add_subdirectory(3rdparty/mrpt_bridge)

# Add dynamic reconfigure api
generate_dynamic_reconfigure_options(
    cfg/Motion.cfg
    cfg/RosInterface.cfg
)

# core
add_library(laser_localization SHARED
    src/mrpt_localization/mrpt_localization.cpp
    src/mrpt_localization/mrpt_localization_core.cpp
    src/mrpt_localization/mrpt_localization_parameters.cpp
    )

target_include_directories(laser_localization
    PUBLIC
    ${THIRD_PARTY_INCLUDE_DIRS}
    include
    )

target_link_libraries(laser_localization
    PUBLIC
    mrpt_bridge
    )

set_target_properties(laser_localization PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SHARED_LIB_PATH}
)

#add_dependencies(${PROJECT_NAME}
#    mrpt_msgs_generate_messages_cpp
#    ${catkin_EXPORTED_TARGETS}
#    mrpt_localization_gencfg
#    )

# executable
add_executable(${PROJECT_NAME}_node
    src/localization_manager.cpp
    src/mrpt_localization_node.cpp
    src/mrpt_localization_node_parameters.cpp
    src/pose_cov_ops.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
    ${GLOG_LIBRARIES}
    laser_localization
    )

#output lib include for other package to depend
catkin_package(
  INCLUDE_DIRS include 3rdparty/mrpt_bridge/include
  LIBRARIES laser_localization mrpt_bridge
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs mrpt_msgs dynamic_reconfigure sensor_msgs 
  DEPENDS MRPT
)

# install(DIRECTORY
#   include/ 
#   3rdparty/mrpt_bridge/include/
#   DESTINATION include/${PROJECT_NAME})
install(DIRECTORY
  3rdparty/mrpt_bridge/include/
  DESTINATION include/)
# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   tutorial
   launch
   ini
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  TARGETS ${PROJECT_NAME}_node laser_localization
  # EXPORT  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)