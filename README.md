# MuJoCo Web

This repository is to compile MuJoCo to WASM (WebAssembly) on Linux Ubuntu 22.04 so it can be run inside a browser.

This is a work-in-progress.

## Steps

To compile MoJoCo to WASM, you must install [Emcripten](https://emscripten.org/index.html), which is a compiler of C++ code that generates WASM code.

Then, run the script `install-deps.sh` in the root folder. The script downloads all required packages, applies the necessary patches and compile MuJoCo.
