cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Infantry)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-address-of-packed-member")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member")

add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

link_directories(${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib)
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} libarm_cortexM4lf_math.a)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 自动收集所有源文件
file(GLOB_RECURSE PROJECT_SOURCES
    ${CMAKE_SOURCE_DIR}/infantry_ws/*.c
    ${CMAKE_SOURCE_DIR}/infantry_ws/*.cpp
)

# 自动收集所有包含头文件的目录
file(GLOB_RECURSE HEADER_FILES
    ${CMAKE_SOURCE_DIR}/infantry_ws/*.h
    ${CMAKE_SOURCE_DIR}/infantry_ws/*.hpp
)
set(INCLUDE_DIRS "")
foreach(_header ${HEADER_FILES})
    get_filename_component(_dir ${_header} PATH)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${PROJECT_SOURCES}
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INCLUDE_DIRS}
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
