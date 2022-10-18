project(tools)

message("-------------- build ${PROJECT_NAME} in ----------")

find_package(OpenCV REQUIRED
    COMPONENTS
    imgproc
    # imgcodecs
    )

message(STATUS "Use OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

add_library(${PROJECT_NAME} SHARED
    src/auto_rotation.cpp
    )

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    PUBLIC
    include
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${OpenCV_LIBRARIES}
    PUBLIC
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
