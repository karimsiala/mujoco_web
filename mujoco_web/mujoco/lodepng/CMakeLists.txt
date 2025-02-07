
cmake_minimum_required(VERSION 3.14)
project(lodepng VERSION 20230907 LANGUAGES CXX)

# Add lodepng as a static library
add_library(lodepng STATIC lodepng.cpp lodepng.h)

# Set include directories as PUBLIC to expose them to consumers
target_include_directories(lodepng PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/lodepng>
)

# Install the library
install(TARGETS lodepng
    EXPORT lodepng_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/lodepng
)

# Create and install the export set
install(EXPORT lodepng_export
    FILE lodepngTargets.cmake
    NAMESPACE lodepng::
    DESTINATION lib/cmake/lodepng
)

# Generate a version file for package configuration
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/lodepngConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Configure the main package config file
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lodepngConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/lodepngConfig.cmake"
    INSTALL_DESTINATION lib/cmake/lodepng
)

# Install the package configuration files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/lodepngConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/lodepngConfigVersion.cmake"
    DESTINATION lib/cmake/lodepng
)

