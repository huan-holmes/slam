project(david_karto)

message("-------------- build ${PROJECT_NAME} in ----------")

# OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(Boost REQUIRED
    COMPONENTS
    thread
    atomic
    )

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

add_library(${PROJECT_NAME} SHARED
    src/Karto.cpp
    src/Mapper.cpp
    )

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${Boost_INCLUDE_DIRS}
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    include
    ${GLOG_INCLUDE_DIR}
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${Boost_LIBRARIES}
    PUBLIC
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
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
