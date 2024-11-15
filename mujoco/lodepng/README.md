# LodePNG library

MuJoCo depends on a library called lodepng: https://github.com/lvandeve/lodepng

lodepng is not a CMake project and this causes some issues whiole compiling with Emscripten. For this reason we fetch the library ourselves and add two custom files to conver the project into a CMake project.

* ./cmake/lodepngConfig.cmake.in
* ./CMakeLists.txt

When we build MuJoCo, it will pick up our library instead of the one fetched by MuJoCo itself.