cmake_minimum_required(VERSION 3.8)
project(ur5_rtde_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(${PROJECT_NAME})

find_package(ur_rtde REQUIRED)

add_executable(ur_rtde_cmake_example
                ur_rtde_test.cc
                )

target_link_libraries(ur_rtde_cmake_example PRIVATE ur_rtde::rtde)

