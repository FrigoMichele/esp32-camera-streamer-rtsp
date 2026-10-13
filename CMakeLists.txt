cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only the components required by the encoder example.
idf_build_set_property(MINIMAL_BUILD ON)
project(camera_rtsp)
