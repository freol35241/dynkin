cmake_minimum_required(VERSION 3.12)
project(dynkin VERSION 0.3.0)


add_library(dynkin INTERFACE)
target_include_directories(dynkin INTERFACE include/)

include(cmake/CPM.cmake)

CPMAddPackage(
  NAME Eigen
  VERSION 3.3.9
  URL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
  # Eigen's CMakelists are not intended for library use
  DOWNLOAD_ONLY YES 
)

if(Eigen_ADDED)
  add_library(Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen INTERFACE ${Eigen_SOURCE_DIR})
endif()

# find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(dynkin INTERFACE Eigen)

add_subdirectory(python)

include(CTest)
enable_testing()
add_subdirectory(tests)
