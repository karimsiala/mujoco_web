**Status:** Successfully built MuJoCo 3.0.0.


# MuJoCo Web

MuJoCo Web is a repository dedicated to compiling MuJoCo (Multi-Joint dynamics with Contact) into WebAssembly (WASM) for seamless execution within web browsers. This project is tailored for Linux Ubuntu 22.04 environments and aims to facilitate interactive physics simulations directly in the browser.

## Inspiration

This project draws inspiration from the following repositories:

https://github.com/stillonearth/MuJoCo-WASM

https://github.com/zalo/mujoco_wasm

These projects provided foundational insights and methodologies that have been adapted and expanded upon to suit the objectives of MuJoCo Web.

## Prerequisites

Before proceeding with the installation and build process, ensure you have the [Emscripten](https://emscripten.org/index.html) compiler installed.

## Installation

Navigate to the root directory of the repository and execute the install-deps.sh script:

```bash
./install-deps.sh
```

This script performs the following actions:

* Downloads all required packages.
* Applies necessary patches to the source code.
* Compiles MuJoCo using the configured environment.



### Key Steps in the Build Process:

**Dependency Management**

MuJoCo relies on the [CCD](https://github.com/danfis/libccd) (Collision Detection) library for its physics computations. By default, MuJoCo fetches and builds the CCD library as part of its CMake build process.

**Patching the CCD Library**

To eliminate external dependencies on the LM (Linear Math) library—which is already included in Emscripten—the CCD library must be patched. This involves modifying the CCD source code to remove references to the external LM library.

**Precompiling the CCD Library**

To streamline the build process and avoid fetching and building CCD from source each time, the CCD library is compiled separately. MuJoCo is then configured to utilize this precompiled version instead of fetching it during the build.

**Configuring MuJoCo to Use the Precompiled CCD Library**

The build scripts are adjusted to prevent MuJoCo from fetching the CCD library automatically. Instead, MuJoCo is instructed to link against the precompiled CCD library, ensuring compatibility and reducing build times.
