# This is a CMakeLists.txt file for building the MuJoCo WASM application.
# 
# To compile and install the executable, run the following commands:
#
# emcmake cmake -Bbuild -H.
# cmake --build build
# cmake --install build

cmake_minimum_required(VERSION 3.22)
project(mujoco_web_root)

# Set the install prefix to the 'dist' directory at the project root.
# This ensures that all installation artifacts are placed in 'project-root/dist/'
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/dist" CACHE PATH "Install path" FORCE)

# Optionally, set the runtime output directory for all targets to 'dist'
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/dist")

# Include the 'mujoco_web' subdirectory
add_subdirectory(mujoco_web)
