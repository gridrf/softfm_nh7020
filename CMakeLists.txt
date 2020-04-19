# CMake definitions for SoftFM

cmake_minimum_required(VERSION 2.8.9)
project(SoftFM)

find_package(Threads)
find_package(PkgConfig)
find_package(ALSA REQUIRED)

# Find libusb
pkg_check_modules(PKG_LIBUSB libusb-1.0)
find_path(LIBUSB_INCLUDE_DIR libusb.h
          HINT ${PKG_LIBUSB_INCLUDE_DIRS}
          PATH_SUFFIXES libusb-1.0)
find_library(LIBUSB_LIBRARY usb-1.0
             HINT ${PKG_LIBUSB_LIBRARY_DIRS})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(libiio 0.9 REQUIRED)

if (LIBIIO_FOUND)
    message(STATUS "LIBIIO_INCLUDE_DIRS: ${LIBIIO_INCLUDE_DIRS}")
    message(STATUS "LIBIIO_LIBRARIES: ${LIBIIO_LIBRARIES}")
    message(STATUS "LIBIIO_DEFINITIONS: ${LIBIIO_DEFINITIONS}")
endif()

find_package(libad9361)

if (LIBAD9361_FOUND)
    message(STATUS "LIBAD9361_INCLUDE_DIRS: ${LIBAD9361_INCLUDE_DIRS}")
    message(STATUS "LIBAD9361_LIBRARIES: ${LIBAD9361_LIBRARIES}")
    message(STATUS "LIBAD9361_DEFINITIONS: ${LIBAD9361_DEFINITIONS}")
    include_directories(${LIBAD9361_INCLUDE_DIRS})
    add_definitions(-DHAS_AD9361_IIO)
endif()

# Compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -ffast-math -ftree-vectorize ${EXTRA_FLAGS}")

add_executable(softfm
    main.cc
    RtlSdrSource.cc
    Filter.cc
    FmDecode.cc
    AudioOutput.cc )

include_directories(
    ${LIBIIO_INCLUDE_DIRS}
    ${LIBAD9361_INCLUDE_DIRS}
    ${ALSA_INCLUDE_DIRS}
    ${EXTRA_INCLUDES} )

target_link_libraries(softfm
    ${CMAKE_THREAD_LIBS_INIT}
    ${LIBIIO_LIBRARIES}
    ${LIBAD9361_LIBRARIES}
    ${ALSA_LIBRARIES}
    ${EXTRA_LIBS} )

install(TARGETS softfm DESTINATION bin)

