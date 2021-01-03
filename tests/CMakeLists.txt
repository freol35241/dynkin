cmake_minimum_required(VERSION 3.12)

add_executable(tests tests.cpp)
target_link_libraries(tests dynkin)

find_package(doctest CONFIG REQUIRED)
target_link_libraries(tests doctest::doctest)
add_test(tests tests)