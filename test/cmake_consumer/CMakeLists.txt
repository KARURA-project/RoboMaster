cmake_minimum_required(VERSION 3.13)

project(RoboMasterCoreConsumer LANGUAGES CXX)

find_package(RoboMasterCore CONFIG REQUIRED)

add_executable(robomaster_consumer main.cpp)
target_link_libraries(robomaster_consumer PRIVATE RoboMaster::Core)
