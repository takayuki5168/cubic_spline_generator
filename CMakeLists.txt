cmake_minimum_required(VERSION 3.0)

project(CubicSplineGenerator CXX C)

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=gnu++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/lib/src/core/*.cpp)
include_directories(LIB_HEADERS ${CMAKE_CURRENT_LIST_DIR}/lib/include/)

# cubic_spline_generator
file(GLOB_RECURSE SOURCES src/*.cpp)
add_library(cubic_spline_generator STATIC ${SOURCES})
target_include_directories(cubic_spline_generator PUBLIC include)

# example
add_executable(main example/main.cpp)
target_link_libraries(main cubic_spline_generator)
