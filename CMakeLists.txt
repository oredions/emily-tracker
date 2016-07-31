cmake_minimum_required(VERSION 2.8)
project(EMILYTracker)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB SOURCES
    *.h
    *.cpp
)
add_executable(EMILYTracker ${SOURCES})
target_link_libraries(EMILYTracker ${OpenCV_LIBS})