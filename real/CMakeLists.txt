cmake_minimum_required(VERSION 3.10)
project(ball_balancer)
set(CMAKE_CXX_STANDARD 17)

set(ORT_DIR ${CMAKE_SOURCE_DIR}/onnxruntime-linux-aarch64-1.20.1)
include_directories(${ORT_DIR}/include)
link_directories(${ORT_DIR}/lib)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA libcamera REQUIRED)

set(SOURCES
    main.cpp
    camera/camera_wrapper.hpp
    camera/frame_stack.hpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    onnxruntime
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    pthread
)