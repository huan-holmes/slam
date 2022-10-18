cmake_minimum_required(VERSION 3.1)
project(mrpt_bridge)

message("-------------- build ${PROJECT_NAME} in ----------")

if (CMAKE_COMPILER_IS_GNUCXX)
  # High level of warnings.
  # The -Wno-long-long is required in 64bit systems when including sytem headers.
  # The -Wno-variadic-macros was needed for Eigen3, StdVector.h
  message(STATUS "in CMAKE_COMPILER_IS_GNUCXX")
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  # Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
  if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif ()
endif ()

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
    src/laser_scan.cpp
    src/map.cpp
    src/pose.cpp
    src/range.cpp
    src/time.cpp
    src/marker_msgs.cpp
    src/beacon.cpp
    src/landmark.cpp
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    include
    )

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    ${MRPT_LIBRARIES}
    )

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SHARED_LIB_PATH}
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

message("-------------- build ${PROJECT_NAME} end ----------")
