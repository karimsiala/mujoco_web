cmake_minimum_required(VERSION 3.22)
project(mujoco_web)

set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/external")
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

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

# Include FetchContent module
include(FetchContent)

# Declare the external project with FetchContent
FetchContent_Declare(
  libccd
  GIT_REPOSITORY git@github.com:danfis/libccd.git
  GIT_TAG v${LIBCCD_VERSION}
  PATCH_COMMAND
    git apply ${CMAKE_CURRENT_SOURCE_DIR}/patches/libccd-${LIBCCD_VERSION}/0001-no-libm.patch
)

# Fetch the content
FetchContent_Populate(libccd)

# Manually add the subdirectory with EXCLUDE_FROM_ALL
add_subdirectory(${libccd_SOURCE_DIR} ${libccd_BINARY_DIR} EXCLUDE_FROM_ALL)

###############################################################################
# mujoco
#

# Define the version of mujoco.
set(MUJOCO_VERSION 3.2.4)

# Append default compiler flags
set(DEFAULT_CXX_FLAGS "-Wno-int-in-bool-context -Wno-newline-eof")
set(DEFAULT_C_FLAGS "-Wno-int-in-bool-context -Wno-newline-eof")

# Define default build options for MuJoCo
option(MUJOCO_BUILD_EXAMPLES "Build MuJoCo examples" OFF)
option(MUJOCO_BUILD_SIMULATE "Build MuJoCo simulate" OFF)
option(MUJOCO_BUILD_TESTS "Build MuJoCo tests" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build MuJoCo Python util tests" OFF)

# Include FetchContent module
include(FetchContent)

# Declare the external project with FetchContent
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY git@github.com:google-deepmind/mujoco.git
  GIT_TAG ${MUJOCO_VERSION}
  PATCH_COMMAND
    git apply ${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0001-no-png.patch &&
    git apply ${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0004-time.patch &&
    git apply ${CMAKE_CURRENT_SOURCE_DIR}/patches/mujoco-${MUJOCO_VERSION}/0005-no-ccd-fetch.patch
)

# Fetch the content
FetchContent_Populate(mujoco)

# Manually add the subdirectory with EXCLUDE_FROM_ALL
add_subdirectory(${mujoco_SOURCE_DIR} ${mujoco_BINARY_DIR} EXCLUDE_FROM_ALL)

###############################################################################
# mujoco_web
#

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# option(JS_ONLY "Compiles to native JS (No WASM)" OFF)

# Specify the directory where executable files are placed after they are built.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/dist")

# Specifies the default directory where the project will be installed when you
# run make install. 
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

add_compile_options(-pthread)
set(EMCC_LINKER_FLAGS "-s ASSERTIONS=1 --bind -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_ES6=1 -s MODULARIZE=1 -s FORCE_FILESYSTEM=1 -s EXPORTED_RUNTIME_METHODS=['FS','MEMFS'] -s EXPORT_NAME=load_mujoco -s EXCEPTION_CATCHING_ALLOWED=['load_from_xml']")
set(CMAKE_REQUIRED_FLAGS "${EMCC_LINKER_FLAGS}")

add_executable(mujoco_web src/main.cpp)
set_target_properties(mujoco_web PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS}")

target_link_libraries(mujoco_web 
  PRIVATE
  ccd
  elasticity
  lodepng
  mujoco
  tinyxml2
  qhullstatic_r
)
