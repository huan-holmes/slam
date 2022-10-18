project(spa_solver)

message("-------------- build ${PROJECT_NAME} in ----------")

find_package(CSparse REQUIRED)
message(STATUS "CSPARSE_INCLUDE_DIR: ${CSPARSE_INCLUDE_DIR}")
message(STATUS "CSPARSE_LIBRARY: ${CSPARSE_LIBRARY}")

add_library(${PROJECT_NAME} SHARED
    src/spa_solver.cpp
    src/csparse.cpp
    src/spa2d.cpp
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CSPARSE_INCLUDE_DIR}
    include
    )

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${CSPARSE_LIBRARY}
    sba
    david_karto
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
