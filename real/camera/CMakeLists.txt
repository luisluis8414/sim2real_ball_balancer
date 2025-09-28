cmake_minimum_required(VERSION 3.10)
project(camera_capture)

set(CMAKE_CXX_STANDARD 17)

# Find libcamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

add_executable(camera_capture camera_capture.cpp)

target_include_directories(camera_capture PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
target_link_libraries(camera_capture ${LIBCAMERA_LIBRARIES})
target_compile_options(camera_capture PRIVATE ${LIBCAMERA_CFLAGS_OTHER})