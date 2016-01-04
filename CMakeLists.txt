cmake_minimum_required (VERSION 2.6)

project (KHRSegwayPC)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions("-Wall -std=c++11")

add_executable (KHRSegwayPC 
  main.cpp
  serial_port.cpp
  command_gen.cpp
)