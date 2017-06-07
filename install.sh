#!/bin/sh

PLUGIN_NAME="$(basename "$(cd $(dirname $0); pwd)")"

if [ "x$VREP_ROOT" = "x" ]; then
    echo "error: \$VREP_ROOT is not set" 1>&2
    exit 1
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

cd "`dirname "$0"`"
make $BUILD_TARGET && \
cp -v "lib$PLUGIN_NAME.$DLEXT" "$INSTALL_TARGET" && \
if [ -f *.lua ]; then cp -v *.lua "$VREP_ROOT/lua/"; fi
exit $?

