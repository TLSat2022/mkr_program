# !!! WARNING !!! AUTO-GENERATED FILE, PLEASE DO NOT MODIFY IT AND USE
# https://docs.platformio.org/page/projectconf/section_env_build.html#build-flags
#
# If you need to override existing CMake configuration or add extra,
# please create `CMakeListsUser.txt` in the root of project.
# The `CMakeListsUser.txt` will not be overwritten by PlatformIO.

cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project("mkr_zero" C CXX)
include_directories(".pio/libdeps/mkrzero/TimerInterrupt/src")
include_directories(".pio/libdeps/mkrzero/Adafruit BMP280 Library")
include_directories(".pio/libdeps/mkrzero/Adafruit BusIO")
include_directories(".pio/libdeps/mkrzero/Adafruit Unified Sensor")
include_directories(".pio/libdeps/mkrzero/LIS3MDL")
include_directories(".pio/libdeps/mkrzero/LPS")
include_directories(".pio/libdeps/mkrzero/LSM6")
include_directories(".pio/libdeps/mkrzero/PID")
include_directories(".pio/libdeps/mkrzero/SD/src")
include_directories(".pio/libdeps/mkrzero/RadioLib/src")
include_directories(".pio/libdeps/mkrzero/WPI430-VMA430 GPS/src")
include_directories(".pio/libdeps/mkrzero/arduino-timer/src")
include_directories(".pio/libdeps/mkrzero/SoftwareSerial")
include(CMakeListsPrivate.txt)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsUser.txt)
include(CMakeListsUser.txt)
endif()

add_custom_target(
    Production ALL
    COMMAND platformio -c clion run "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    Debug ALL
    COMMAND platformio -c clion debug "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(Z_DUMMY_TARGET ${SRC_LIST})
