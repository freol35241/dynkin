cmake_minimum_required(VERSION 3.12)

CPMAddPackage(
  NAME pybind11
  GITHUB_REPOSITORY pybind/pybind11
  GIT_TAG v2.6.1
)

pybind11_add_module(pydynkin dynkin.cpp)
target_link_libraries(pydynkin PRIVATE dynkin)