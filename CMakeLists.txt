cmake_minimum_required(VERSION 3.16)
project(osciladores LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
add_executable(osciladores src/main.cpp src/config.cpp src/dynamics.cpp src/recorder.cpp)
target_include_directories(osciladores PRIVATE include)
# Host API baseline matches OpenCL C 1.1 kernels for older Mesa/Clover devices.
target_compile_definitions(osciladores PRIVATE CL_TARGET_OPENCL_VERSION=110 RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(osciladores PRIVATE OpenCL::OpenCL OpenGL::GL OpenGL::GLU GLUT::GLUT)
target_compile_options(osciladores PRIVATE -Wall -Wextra -Wpedantic)
enable_testing()
add_test(NAME physics COMMAND osciladores --self-test)

# Python is only needed for parser tests, not for the simulator.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME parser COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/parser.py $<TARGET_FILE:osciladores>)
endif()
foreach(config ejemplo onda_forzada)
  add_test(NAME config_${config} COMMAND osciladores
    ${CMAKE_CURRENT_SOURCE_DIR}/parameters/${config}.cfg --validate)
endforeach()
