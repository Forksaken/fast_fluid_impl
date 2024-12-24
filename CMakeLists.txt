cmake_minimum_required(VERSION 3.8)
project(fast_fluid_impl)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_executable(fast_fluid fluid_optimised.cpp
        ThreadPool.hpp)
target_compile_options(fast_fluid PRIVATE -O0)

add_executable(original fluid_og.cpp)
target_compile_options(original PRIVATE -O0)
