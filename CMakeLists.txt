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

# cubic_spline_generator
file(GLOB_RECURSE SOURCES src/*.cpp)
add_library(cubic_spline_generator STATIC ${SOURCES})
target_include_directories(cubic_spline_generator PUBLIC include)

# example
add_executable(example example/main.cpp)
target_link_libraries(example cubic_spline_generator)
