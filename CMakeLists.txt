# This is a CMakeLists.txt file for building the MuJoCo WASM application.
# 
# To compile and install the executable, run the following commands:
#
# emcmake cmake -Bbuild -H.
# cmake --build build
# cmake --install build

cmake_minimum_required(VERSION 3.22)
project(mujoco_web)

# Check if EMSDK environment variable is defined.
if(NOT DEFINED ENV{EMSDK})
    message(FATAL_ERROR "EMSDK environment variable is not set. Please set EMSDK to the root of your Emscripten SDK installation.")
endif()

# Get EMSDK path from environment variable
set(EMSDK_PATH $ENV{EMSDK})

# Verify that EMSDK_PATH exists and contains necessary directories
if(NOT EXISTS "${EMSDK_PATH}/upstream/emscripten")
    message(FATAL_ERROR "EMSDK path '${EMSDK_PATH}' does not contain 'upstream/emscripten'. Please verify your EMSDK path.")
endif()

# Specifies the default directory where the project will be installed when you
# run make install. 
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}//public/wasm")

# Optionally, set the runtime output directory for all targets to 'dist'
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/public/wasm")

# Include the 'mujoco_web' subdirectory
add_subdirectory(mujoco_web)
