cmake_minimum_required(VERSION 3.5)

set(VERSION "0.0")

project(kalman_imu_model)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

include_directories($(PROJECT_SOURCE_DIR)/include/)

# All libraries required goes here
set(KLIB_SRCS
    include/math/kmath.cpp
    include/sensor/sensor.cpp
)

# Add and libraries
add_library(lib ${KLIB_SRCS})

# linking and executable generation
add_executable(sim main.cpp)
target_link_libraries(sim lib)
