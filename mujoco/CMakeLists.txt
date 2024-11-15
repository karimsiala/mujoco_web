# This is a CMakeLists.txt file for building the mujoco_wasm project.
# 
# First, it checks if Emscripten is installed by inspecting the  EMSDK
# environment variable.
# 
# Second, it fetches all required dependencies using FetchContent and applies
# patches.
#
# Finally, it compiles the project using Emscripten.

# Ensure this CMakeLists.txt is included by a parent project.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR "Please run CMake from the top-level directory.")
endif()

set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/external")
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

# Generate the compile_commands.json that can be used to configure clangd.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include FetchContent module
include(FetchContent)


###############################################################################
# libccd
#

# Define the version of libccd
set(LIBCCD_VERSION 2.1)

# Define default build options for libccd
set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "Enable double precision" FORCE)
set(CCD_HIDE_ALL_SYMBOLS ON CACHE BOOL "Hide all symbols in CCD" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build testing" FORCE)

message(STATUS "Fetching libccd ${LIBCCD_VERSION}...")
FetchContent_Declare(
  libccd
  GIT_REPOSITORY https://github.com/danfis/libccd.git
  GIT_TAG v${LIBCCD_VERSION}
  PATCH_COMMAND
    git apply "${CMAKE_CURRENT_SOURCE_DIR}/patches/libccd-${LIBCCD_VERSION}/0001-no-libm.patch"
)
FetchContent_Populate(libccd)

# Manually add the subdirectory with EXCLUDE_FROM_ALL
add_subdirectory(${libccd_SOURCE_DIR} ${libccd_BINARY_DIR} EXCLUDE_FROM_ALL)

###############################################################################
# lodepng
#

message(STATUS "Fetching lodepng...")
FetchContent_Declare(
  lodepng
  GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
)
FetchContent_Populate(lodepng)

# Create cmake folder.
file(MAKE_DIRECTORY "${lodepng_SOURCE_DIR}/cmake")

# Copy CMakeLists.txt
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/lodepng/CMakeLists.txt" 
DESTINATION "${lodepng_SOURCE_DIR}/")

# Copy lodepngConfig.cmake.in
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/lodepng/cmake/lodepngConfig.cmake.in" 
DESTINATION "${lodepng_SOURCE_DIR}/cmake")

# Manually add the subdirectory with EXCLUDE_FROM_ALL
add_subdirectory(${lodepng_SOURCE_DIR} ${lodepng_BINARY_DIR} EXCLUDE_FROM_ALL)

###############################################################################
# mujoco
#

# Define the version of mujoco.
set(MUJOCO_VERSION 3.2.5)

# Set global compiler flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-int-in-bool-context -Wno-newline-eof -pthread -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-in-bool-context -Wno-newline-eof -pthread -O3")

# Define default build options for MuJoCo
option(MUJOCO_BUILD_EXAMPLES "Build MuJoCo examples" OFF)
option(MUJOCO_BUILD_SIMULATE "Build MuJoCo simulate" OFF)
option(MUJOCO_BUILD_TESTS "Build MuJoCo tests" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build MuJoCo Python util tests" OFF)

message(STATUS "Fetching  mujoco ${MUJOCO_VERSION}...")
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG ${MUJOCO_VERSION}
  PATCH_COMMAND
  #  git apply --verbose "${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0001-no-png.patch" &&
    git apply --verbose "${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0004-time.patch" &&
    git apply --verbose "${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0005-no-ccd-fetch.patch"
  #   git apply --verbose "${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0006-private-lodepng.patch"
)
FetchContent_Populate(mujoco)

# Manually add the subdirectory with EXCLUDE_FROM_ALL
add_subdirectory(${mujoco_SOURCE_DIR} ${mujoco_BINARY_DIR} EXCLUDE_FROM_ALL)

###############################################################################
# mujoco_wasm
#

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Linker flags
set(EMCC_LINKER_FLAGS "")
string(APPEND EMCC_LINKER_FLAGS
  "--bind "                                           # Automatically bind C++ classes and functions to JavaScript
  "-g "                                               # Enable debug information for easier debugging
  "-s WASM=1 "                                        # Generate WebAssembly output
  "-s STACK_OVERFLOW_CHECK "
  "-s STACK_SIZE=5MB "  
  "-s INITIAL_MEMORY=134217728 "                      # Set initial memory (128MB as an example)
  "-s MAXIMUM_MEMORY=536870912 "                      # Set maximum memory (512MB as an example) 
  "-s ALLOW_MEMORY_GROWTH=1 "                         # Allow WebAssembly memory to grow dynamically if needed                                         
  "-s USE_PTHREADS=1 "                                # Generate additional .worker js file.
  "-s ENVIRONMENT=web,worker "
  "-s PTHREAD_POOL_SIZE=16 "                          # Allow the thread pool size to be set at runtime
  "-s ASSERTIONS=2 "                                  # Enable runtime assertions for catching common errors
  "-s EXPORT_ES6=1 "                                  # Export the WebAssembly module in ES6 format
  "-s MODULARIZE=1 "                                  # Wrap the WebAssembly module in a function for encapsulation
  "-s FORCE_FILESYSTEM=1 "                            # Include the virtual filesystem (VFS) for file operations
  "-s EXPORTED_RUNTIME_METHODS=['FS','MEMFS'] "       # Export 'FS' and 'MEMFS' for in-memory file operations
  "-s DISABLE_EXCEPTION_CATCHING=0 "                  # Enable exception catching
  "-s EXPORT_NAME=load_mujoco "                       # Set the export name of the module to 'load_mujoco'
)
set(CMAKE_REQUIRED_FLAGS "${EMCC_LINKER_FLAGS}")

# Add the executable target.
add_executable(mujoco_wasm src/main.cpp)

# The following two lines solve this issue: https://github.com/clangd/clangd/issues/1621
set(EMSCRIPTEN_SYSROOT "${EMSDK_PATH}/upstream/emscripten/cache/sysroot")
set_target_properties(mujoco_wasm PROPERTIES COMPILE_FLAGS "-target wasm32-unknown-emscripten --sysroot=${EMSCRIPTEN_SYSROOT}")
set_target_properties(mujoco_wasm PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS}")

target_include_directories(mujoco_wasm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(mujoco_wasm PRIVATE
  mujoco
)

# Invoking a custom command to fix an issue in thew generated wasm .js file.
set(GENERATED_FILE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/mujoco_wasm.js)
add_custom_command(
    TARGET mujoco_wasm
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -Dfilepath="${GENERATED_FILE}" -P "${CMAKE_CURRENT_LIST_DIR}/cmake/fixWasm.cmake"
)



