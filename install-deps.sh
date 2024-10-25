#!/bin/bash
set -e

function install-all {
    # Export environment variables.

    export THREADS=16
    export CURRENT_DIR=$(pwd)

    export DEPS_DIR=$CURRENT_DIR/deps
    if [ ! -d "$DEPS_DIR" ]; then
        mkdir -p "$DEPS_DIR"
    fi

    export SRC_DIR=$DEPS_DIR/src
    if [ ! -d "$SRC_DIR" ]; then
        mkdir -p "$SRC_DIR"
    fi

    export DST_DIR=$DEPS_DIR/dst
    if [ ! -d "$DST_DIR" ]; then
        mkdir -p "$DST_DIR"
    fi

    export BUILD_DIR=$DEPS_DIR/build
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir -p "$BUILD_DIR"
    fi

    # Compile and install all dependencies.

    source $CURRENT_DIR/install-deps/libccd/install-libccd.sh
    source $CURRENT_DIR/install-deps/mujoco/install-mujoco.sh

    # Print all environment variables.

    echo
    echo "export C_INCLUDE_PATH=$C_INCLUDE_PATH"
    echo
    echo "export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH"
    echo
    echo "export LIBRARY_PATH=$LIBRARY_PATH\n"
    echo
    echo "export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
    echo
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
    echo
    echo "export PATH=$PATH"
    echo
    echo "export DST_DIR=$DEPS_DIR/dst"
    echo
}

install-all
