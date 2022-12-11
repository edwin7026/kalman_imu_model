cmake_minimum_required(VERSION 3.5)

project(kalman_imu_model)

include_directories($(PROJECT_SOURCE_DIR)/include/)
add_executable(sim main.cpp)
