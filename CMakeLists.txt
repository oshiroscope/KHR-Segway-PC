cmake_minimum_required (VERSION 2.6)

project (KHRSegwayPC)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_definitions("-Wall -std=c++11 -I/usr/local/include/aruco")

add_executable (KHRSegwayPC 
  main.cpp
  serial_port.cpp
  command_gen.cpp
  key_input.cpp
  motion.cpp
  odometry.cpp
  monitor.cpp
)

target_link_libraries(
  KHRSegwayPC
  pthread
  ${OpenCV_LIBS}
  aruco
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
)