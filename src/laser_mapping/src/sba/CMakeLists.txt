# sparse bundle adjustment
project(sba)

message("-------------- build ${PROJECT_NAME} in ----------")

find_package(suitesparse QUIET)
if (NOT SuiteSparse_FOUND)
  set(SuiteSparse_LIBRARIES blas lapack cholmod cxsparse)
  message(STATUS "SuiteSparse_LIBRARIES: ${SuiteSparse_LIBRARIES}")
else ()
  message(STATUS "SuiteSparse_INCLUDE_DIRS: ${SuiteSparse_INCLUDE_DIRS}")
  message(STATUS "SuiteSparse_LIBRARIES: ${SuiteSparse_LIBRARIES}")
endif ()

add_definitions(-DSBA_CHOLMOD)

# SBA library
add_library(${PROJECT_NAME}
    src/sba.cpp
    src/spa.cpp
    src/spa2d.cpp
    src/csparse.cpp
    src/proj.cpp
    src/node.cpp
    src/sba_file_io.cpp
    )

target_include_directories(${PROJECT_NAME}
    PRIVATE
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${SuiteSparse_INCLUDE_DIRS}
    include
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    PUBLIC
    ${SuiteSparse_LIBRARIES}
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
