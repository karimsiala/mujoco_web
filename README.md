# MuJoCo Web

This repository is to compile MuJoCo to WASM (WebAssembly) on Linux Ubuntu 22.04 so it can be run inside a browser.

This is a work-in-progress, inspired by:

https://github.com/stillonearth/MuJoCo-WASM

https://github.com/zalo/mujoco_wasm

## Issues

MuJoCo requires the [CCD](https://github.com/danfis/libccd) library and it fetches it out automatically as part of the CMake building process. The CCD library must be patched in order to remove the external dependency to the LM library, which is already included in EmScripten.

We must either instruct MuJoCo to use our patched CCD library, or we must patch the one donwloaded automatically by MuJoCo.

## Steps

To compile MoJoCo to WASM, you must install [Emscripten](https://emscripten.org/index.html), which is a compiler of C++ code that generates WASM code.

Then, run the script `install-deps.sh` in the root folder. The script downloads all required packages, applies the necessary patches and compile MuJoCo.
