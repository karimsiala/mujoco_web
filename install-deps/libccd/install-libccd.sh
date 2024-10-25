#!/bin/bash
set -e

function compile-libccd {

    TAG=2.1
    PATCHES_DIR=$CURRENT_DIR/install-deps/libccd/patch

    # Where C finds include files at compile time.
    export C_INCLUDE_PATH=$DST_DIR/libccd/include:$C_INCLUDE_PATH

    # Where C++ finds include files at compile time.
    export CPLUS_INCLUDE_PATH=$DST_DIR/libccd/include:$CPLUS_INCLUDE_PATH
    
    # Where to find libraries at compile time.
    export LIBRARY_PATH=$DST_DIR/libccd/lib:$LIBRARY_PATH

    # Where to find libraries at runtime.
    export LD_LIBRARY_PATH=$DST_DIR/libccd/lib:$LD_LIBRARY_PATH

    # Where CMake find additional packages.
    export CMAKE_PREFIX_PATH=$DST_DIR/libccd:$CMAKE_PREFIX_PATH

    if [ -d "$DST_DIR/libccd" ]; then
        echo "Library libccd already installed."
        return
    fi

    cd $SRC_DIR
    if [ ! -f "libccd-$TAG.zip" ]; then
        wget https://github.com/danfis/libccd/archive/refs/tags/v$TAG.zip -O libccd-$TAG.zip
    fi
    cp libccd-$TAG.zip $BUILD_DIR
    cd $BUILD_DIR
    rm -rf libccd-$TAG
    unzip libccd-$TAG.zip
    rm -rf libccd-$TAG.zip
    cd libccd-$TAG

    # Apply patches.
    apply_patches "$PATCHES_DIR"

    # Build and install.
    # Tests are disabled because they need librt, not available in Emscripten.
    emcmake cmake -Bbuild -H. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$DST_DIR/libccd \
      -DENABLE_DOUBLE_PRECISION=ON \
      -DCCD_HIDE_ALL_SYMBOLS=ON \
      -DBUILD_SHARED_LIBS=OFF \
      -DBUILD_TESTING=OFF

    cmake --build build
    cmake --install build
}

compile-libccd

