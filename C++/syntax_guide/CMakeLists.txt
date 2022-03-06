cmake_minimum_required(VERSION 3.5)
project(simple_topic)

include_directories(include)

add_executable(main src/main.cpp src/distance.cpp)