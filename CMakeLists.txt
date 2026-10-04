cmake_minimum_required(VERSION 3.13)

project(RoboMasterCore VERSION 0.2.0 LANGUAGES CXX)

option(ROBOMASTER_BUILD_TESTS "Build the board-independent tests" ON)

include(GNUInstallDirs)

set(CANBRIDGE_INCLUDE_DIR "" CACHE PATH
    "Directory containing CANBridge.h")
if(NOT CANBRIDGE_INCLUDE_DIR)
    find_path(CANBRIDGE_INCLUDE_DIR
        NAMES CANBridge.h
        HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../CANBridge/src")
endif()
if(NOT CANBRIDGE_INCLUDE_DIR)
    message(FATAL_ERROR
        "CANBridge.h was not found. Set CANBRIDGE_INCLUDE_DIR to the CANBridge src directory.")
endif()

add_library(robomaster_core
    src/robomaster/Diagnostics.cpp
    src/robomaster/Motor.cpp
    src/robomaster/MotorBus.cpp
    src/robomaster/MotorState.cpp
    src/robomaster/Pid.cpp
    src/robomaster/Protocol.cpp)

add_library(RoboMaster::Core ALIAS robomaster_core)

target_include_directories(robomaster_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CANBRIDGE_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(robomaster_core PUBLIC cxx_std_11)
set_target_properties(robomaster_core PROPERTIES
    CXX_EXTENSIONS OFF
    EXPORT_NAME Core)

install(TARGETS robomaster_core
    EXPORT RoboMasterCoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY src/robomaster
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp")
install(FILES src/RoboMasterCore.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/RoboMasterCoreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/RoboMasterCoreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboMasterCore)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/RoboMasterCoreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(EXPORT RoboMasterCoreTargets
    FILE RoboMasterCoreTargets.cmake
    NAMESPACE RoboMaster::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboMasterCore)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/RoboMasterCoreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/RoboMasterCoreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboMasterCore)

if(ROBOMASTER_BUILD_TESTS)
    include(CTest)

    function(robomaster_add_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} PRIVATE robomaster_core)
        target_compile_features(${name} PRIVATE cxx_std_11)
        set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
            target_compile_options(${name} PRIVATE
                -Wall -Wextra -Wpedantic -Werror)
        endif()
        add_test(NAME ${name} COMMAND ${name})
    endfunction()

    robomaster_add_test(test_portable_boundary test/test_portable_boundary.cpp)
    target_compile_definitions(test_portable_boundary PRIVATE ARDUINO_ARCH_ESP32=1)
    robomaster_add_test(test_control test/test_control.cpp)
    robomaster_add_test(test_diagnostics test/test_diagnostics.cpp)
    robomaster_add_test(test_motor test/test_motor.cpp)
    robomaster_add_test(test_motor_bus test/test_motor_bus.cpp)
    robomaster_add_test(test_motor_state test/test_motor_state.cpp)
    robomaster_add_test(test_pid test/test_pid.cpp)
    robomaster_add_test(test_properties test/test_properties.cpp)
    robomaster_add_test(test_protocol test/test_protocol.cpp)
    robomaster_add_test(test_simulation test/test_simulation.cpp)
endif()
