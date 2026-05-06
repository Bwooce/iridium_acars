# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Shared cross-board components live in ../common/. A future worker P4
# project (sibling of p4-usb-host) will set the same EXTRA_COMPONENT_DIRS
# to share decoders, types, and protocol code without duplication.
list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p4-usb-host)
