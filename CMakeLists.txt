cmake_minimum_required(VERSION 3.12)
project(dynkin VERSION 0.3.0)


add_library(dynkin INTERFACE)
target_include_directories(dynkin INTERFACE include/)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(dynkin INTERFACE Eigen3::Eigen)


include(CTest)
enable_testing()
add_subdirectory(tests)

