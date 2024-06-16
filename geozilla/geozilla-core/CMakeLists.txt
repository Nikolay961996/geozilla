cmake_minimum_required(VERSION 3.15)

project(geozilla-core LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/external/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(PCL_DIR "${CMAKE_CURRENT_LIST_DIR}/external/vcpkg/installed/x64-windows/share/pcl")
set(Eigen3_DIR "${CMAKE_CURRENT_LIST_DIR}/external/vcpkg/installed/x64-windows/share/eigen3")

add_subdirectory(external)
add_subdirectory(source)
