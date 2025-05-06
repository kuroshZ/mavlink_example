cmake_minimum_required(VERSION 3.12)
project(mavlink_cpp_project)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
# Find required packages
find_package(fmt REQUIRED)
set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/c_library_v2 )
# Add Asio

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(lib/asio-1.30.2/include)


# Add MAVLink
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/c_library_v2)
include_directories(${MAVLINK_INCLUDE_DIR}/common)

# Add source files
add_executable(mavlink_client src/main.cpp src/helpers.cpp)

target_include_directories(mavlink_client PRIVATE SYSTEM ${MAVLINK_INCLUDE_DIR}/common)

# Link libraries
target_link_libraries(mavlink_client PRIVATE fmt::fmt)

# Add compiler flags
target_compile_options(mavlink_client PRIVATE -Wall -Wextra -Wpedantic)
