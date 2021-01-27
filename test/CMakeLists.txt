cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(dynkinTests LANGUAGES CXX)


# --- Import tools ----

include(../cmake/tools.cmake)

# ---- Dependencies ----

include(../cmake/CPM.cmake)

CPMAddPackage(
  NAME doctest
  GITHUB_REPOSITORY onqtam/doctest
  GIT_TAG 2.4.0
)

CPMAddPackage(
    NAME dynkin 
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..
)

# ---- Create binary ----
add_executable(dynkinTests tests.cpp)
target_link_libraries(dynkinTests doctest dynkin::dynkin)


# ---- Add dynkinTests ----

enable_testing()

# Note: doctest and similar testing frameworks can automatically configure CMake tests For other
# testing frameworks add the tests target instead: ADD_TEST(GreeterTests GreeterTests)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(dynkinTests)
