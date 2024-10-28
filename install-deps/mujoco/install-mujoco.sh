#!/bin/bash
set -e

function compile-libccd {

    TAG=3.2.0
    PATCHES_DIR=$CURRENT_DIR/install-deps/mujoco/patch-$TAG
    INSTALL_PREFIX=$DST_DIR/mujoco-$TAG

    # Where C finds include files at compile time.
    export C_INCLUDE_PATH=$INSTALL_PREFIX/include:$C_INCLUDE_PATH

    # Where C++ finds include files at compile time.
    export CPLUS_INCLUDE_PATH=$INSTALL_PREFIX/include:$CPLUS_INCLUDE_PATH
    
    # Where to find libraries at compile time.
    export LIBRARY_PATH=$INSTALL_PREFIX/lib:$LIBRARY_PATH

    # Where to find libraries at runtime.
    export LD_LIBRARY_PATH=$DINSTALL_PREFIX/lib:$LD_LIBRARY_PATH

    # Where CMake find additional packages.
    export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:$CMAKE_PREFIX_PATH

    if [ -d "$INSTALL_PREFIX" ]; then
        echo "Library mujoco $TAG already installed."
        return
    fi

    cd $SRC_DIR
    if [ ! -f "mujoco-$TAG.zip" ]; then
        wget https://github.com/google-deepmind/mujoco/archive/refs/tags/$TAG.zip -O mujoco-$TAG.zip
    fi
    cp mujoco-$TAG.zip $BUILD_DIR
    cd $BUILD_DIR
    rm -rf mujoco-$TAG
    unzip mujoco-$TAG.zip
    rm -rf mujoco-$TAG.zip
    cd mujoco-$TAG

    # Apply patches.
    apply_patches "$PATCHES_DIR"

    # Build and install.
    emcmake cmake -Bbuild -H. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
      -DCMAKE_CXX_FLAGS="-Wno-int-in-bool-context -Wno-newline-eof" \
      -DCMAKE_C_FLAGS="-Wno-int-in-bool-context -Wno-newline-eof" \
      -DMUJOCO_BUILD_EXAMPLES=OFF \
      -DMUJOCO_BUILD_SIMULATE=OFF \
      -DMUJOCO_BUILD_TESTS=OFF \
      -DMUJOCO_TEST_PYTHON_UTIL=OFF
    cmake --build build
    cmake --install build
}

compile-libccd
