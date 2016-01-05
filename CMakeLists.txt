cmake_minimum_required (VERSION 2.6)

project (KHRSegwayPC)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)

add_definitions("-Wall -std=c++11 -I/usr/local/include/aruco")

add_executable (KHRSegwayPC 
  main.cpp
  serial_port.cpp
  command_gen.cpp
  key_input.cpp
  motion.cpp
)

target_link_libraries(
  KHRSegwayPC
  pthread
  ${OpenCV_LIBS}
  aruco
)