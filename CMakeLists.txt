cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(find_obstacles CXX)
set(EXECUTABLE_NAME "find_obstacles")


#-------------------------------------------------------------------------------
# Compiler config
#-------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    add_compile_options("-Wall")
    add_compile_options("-Wextra")
    add_compile_options("-Wconversion")
    add_compile_options("-pedantic")
    add_compile_options("-Wfatal-errors")
    add_compile_options("-lm")
endif()


#-------------------------------------------------------------------------------
# target source files
#-------------------------------------------------------------------------------
set(SRC_PATH "./find_obstacles")
file(GLOB SRC_FILES
    "${SRC_PATH}/include/find_obstacles/*.hpp"
    "${SRC_PATH}/src/*.cpp"
)
if(SRC_FILES)
    message("-- Found source files at ${SRC_PATH}")
endif()
include_directories("${SRC_PATH}/include")


#-------------------------------------------------------------------------------
# target PCL
#-------------------------------------------------------------------------------
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(${EXECUTABLE_NAME} ${SRC_FILES})
target_link_libraries(${EXECUTABLE_NAME} ${PCL_LIBRARIES})
