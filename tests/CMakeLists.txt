cmake_minimum_required(VERSION 3.12)

add_executable(tests tests.cpp)

CPMAddPackage(
  NAME doctest
  GITHUB_REPOSITORY onqtam/doctest
  GIT_TAG 2.3.2
)

target_link_libraries(tests dynkin doctest)
add_test(tests tests)