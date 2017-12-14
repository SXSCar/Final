cmake_minimum_required(VERSION 2.8)

PROJECT(LaneDetection_Demo)

FIND_PACKAGE(OpenCV REQUIRED)

SET(CMAKE_C_COMPILER g++)

SET(CMAKE_EXE_LINKER_FLAGS "-L/home/pi/Desktop/libGPIO.a -lpthread -lGPIO -lwiringPi")
SET(INC_DIR ./)
include_directories(${INC_DIR})

SET(SRC
 main.cpp
)

SET(PROJECT_NAME
 demo
)

ADD_EXECUTABLE(${PROJECT_NAME} ${SRC})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${OpenCV_LIBS})
