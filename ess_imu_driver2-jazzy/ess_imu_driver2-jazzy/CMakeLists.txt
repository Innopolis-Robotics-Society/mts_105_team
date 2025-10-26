cmake_minimum_required(VERSION 3.5)
project(ess_imu_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# NOTE: Refer to the README_src.md inside the src folder for more details about the C source files

################################################################
# These two settings are typically the only changes required by
# the user before building this ROS node

# 1. Select NONE for standard PC, or RPI for RaspberryPi
set(PLATFORM "NONE")
#set(PLATFORM "RPI")

# 2. Select serial interface type UART or SPI
set(INTERFACE "UART")
#set(INTERFACE "SPI")

###############################################################

message(STATUS "---- Building for platform: ${PLATFORM}")
message(STATUS "---- Building for interface: ${INTERFACE}")

# When SPI interface selected, platform must be RPI (since PCs do not natively support SPI)
if (INTERFACE STREQUAL "SPI")
  set(PLATFORM "RPI")
  message(STATUS "---- SPI selected forcing platform: ${PLATFORM}")
endif()

# Define macros variables for compilation
add_definitions(-D${PLATFORM})
add_definitions(-D${INTERFACE})

# Create file list for C libraries based on PLATFORM
if (PLATFORM STREQUAL "RPI")
  set(lib_sources
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
  )
elseif (PLATFORM STREQUAL "NONE")
  set(lib_sources
    src/hcl_linux.c
    src/hcl_gpio.c
  )
else()
  message(FATAL_ERROR "**** Invalid Platform")
endif()

# Create file list for C libraries based on INTERFACE
if (INTERFACE STREQUAL "SPI")
  set(lib_sources ${lib_sources}
    src/hcl_spi_rpi.c
    src/sensor_epsonCommon.c
    src/sensor_epsonSpi.c
  )
elseif (INTERFACE STREQUAL "UART")
  set(lib_sources ${lib_sources}
    src/hcl_uart.c
    src/sensor_epsonCommon.c
    src/sensor_epsonUart.c
  )
else()
  message(FATAL_ERROR "**** Invalid Interface")
endif()

# Declare library for Epson IMU functions
add_library(ess_imu_driver2_lib
  ${lib_sources}
)

# Link any external libraries to Epson IMU Library
if (PLATFORM STREQUAL "RPI")
  # Determine location of wiringPi library on the host system
  # Required for building on Raspberry Pi platform
  find_library(wiringPi_LIB NAMES wiringPi)

  target_link_libraries(ess_imu_driver2_lib
    ${wiringPi_LIB}
  )
endif()


# Declare a C++ executable ROS node from source file
if (INTERFACE STREQUAL "SPI")
add_executable(ess_imu_driver2_node
  src/epson_imu_spi_ros2_node.cpp
)
elseif (INTERFACE STREQUAL "UART")
add_executable(ess_imu_driver2_node
  src/epson_imu_uart_ros2_node.cpp
)
else()
  message(FATAL_ERROR "**** Invalid Interface")
endif()


# Link C libraries to C++ executable ROS node
target_link_libraries(ess_imu_driver2_node
  ess_imu_driver2_lib
  crypt
  rt
  pthread
)

# Link C++ Library to C++ executable ROS node
ament_target_dependencies(ess_imu_driver2_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS ess_imu_driver2_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

ament_package()
