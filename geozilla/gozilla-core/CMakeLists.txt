cmake_minimum_required(VERSION 3.15)

project(geozilla-core LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(external)
add_subdirectory(source)
