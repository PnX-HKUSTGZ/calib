cmake_minimum_required(VERSION 3.16)

project(Camera_Calibration)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIR}
        ${ROOT}/IndustrialCamera/include
        ${ROOT}/Data
        ${ROOT}/Logging/include
        )

file(GLOB_RECURSE src_lists
        ${ROOT}/IndustrialCamera/src/*
        ${ROOT}/Logging/src/*
        )

add_executable(Camera_Calibration camera_calibration.cpp ${src_lists})


#dynamic link libraries

target_link_libraries(Camera_Calibration ${OpenCV_LIBS}
        ${ROOT}/IndustrialCamera/libs/libFormatConversion.so
        ${ROOT}/IndustrialCamera/libs/libMediaProcess.so
        ${ROOT}/IndustrialCamera/libs/libMvCameraControl.so
        ${ROOT}/IndustrialCamera/libs/libMVRender.so
        ${ROOT}/IndustrialCamera/libs/libMvUsb3vTL.so
        )