cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(pydynkin LANGUAGES CXX)


# --- Import tools ----

include(../cmake/tools.cmake)

# ---- Dependencies ----

include(../cmake/CPM.cmake)

CPMAddPackage(
  NAME pybind11
  GITHUB_REPOSITORY pybind/pybind11
  GIT_TAG v2.6.1
)

CPMAddPackage(
    NAME dynkin 
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..
)

pybind11_add_module(${PROJECT_NAME} dynkin.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE dynkin)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME _dynkin)
install(TARGETS ${PROJECT_NAME} DESTINATION .) 