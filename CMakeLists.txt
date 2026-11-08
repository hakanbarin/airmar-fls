# Builds and registers the self test. The sensor itself needs no build system:
# FLS.hpp and FLS.cpp drop straight into whatever already compiles your project.
cmake_minimum_required(VERSION 3.16)
project(fls LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(fls STATIC FLS.cpp)
target_include_directories(fls PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(MSVC)
  target_compile_options(fls PRIVATE /W4 /WX)
else()
  target_compile_options(fls PRIVATE -Wall -Wextra -Werror)
endif()

enable_testing()
add_executable(test_fls test_fls.cpp)
target_link_libraries(test_fls PRIVATE fls)
add_test(NAME fls_self_test COMMAND test_fls)
