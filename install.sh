#!/bin/sh

set -e

cd "`dirname "$0"`"

PLUGIN_NAME="$(basename "$(pwd)")"

if [ "x$VREP_ROOT" = "x" ]; then
    # see if we can determine it automatically
    # (i.e. the plugin dir is in $VREP_ROOT/programming/$dir)
    VREP_ROOT="$(cd ../.. ; pwd)"
    if [ ! -d "$VREP_ROOT/programming/include" ]; then
        echo "error: \$VREP_ROOT is not set" 1>&2
        exit 1
    fi
fi

if [ "`uname`" = "Darwin" ]; then
    INSTALL_TARGET="$VREP_ROOT/vrep.app/Contents/MacOS/"
    DLEXT=dylib
else
    INSTALL_TARGET="$VREP_ROOT"
    DLEXT=so
fi

if [ "x$BUILD_TARGET" = "x" ]; then
    BUILD_TARGET=debug
fi

LIBRARY="lib$PLUGIN_NAME.$DLEXT"

if [ -f CMakeLists.txt ]; then
    # plugin uses cmake
    mkdir -p build
    cd build
    cmake ..
    cmake --build .
    cd ..
    LIBRARY="build/$LIBRARY"
elif [ -f $PLUGIN_NAME.pro ]; then
    # plugin uses qmake
    qmake $PLUGIN_NAME.pro
    make $BUILD_TARGET
elif [ -f makefile ]; then
    # plugin uses make
    make $BUILD_TARGET
else
    echo "Unable to figure out the build system of $PLUGIN_NAME"
    exit 1
fi

cp -v "$LIBRARY" "$INSTALL_TARGET"
if [ -f *.lua ]; then cp -v *.lua "$VREP_ROOT/lua/"; fi

