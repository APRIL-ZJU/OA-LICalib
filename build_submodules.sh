#!/usr/bin/env bash

MYPWD=$(pwd)

#https://www.cnblogs.com/robinunix/p/11635560.html
# set -x 
# set -e 
set -x
set -e

BUILD_TYPE=RelWithDebInfo

if [ -n "$1" ]; then
BUILD_TYPE=$1
fi

# https://stackoverflow.com/a/45181694
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`

NUM_PARALLEL_BUILDS=$NUM_CORES

CXX_MARCH=native

EIGEN_DIR="$MYPWD/thirdparty/eigen"

COMMON_CMAKE_ARGS=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
    -DCMAKE_CXX_FLAGS="-march=$CXX_MARCH -O3 -Wno-deprecated-declarations -Wno-null-pointer-arithmetic -Wno-unknown-warning-option -Wno-unused-function" #  -Wno-int-in-bool-context
)

git submodule sync --recursive
git submodule update --init --recursive

BUILD_CERES=thirdparty/build-ceres-solver
rm -rf "$BUILD_CERES"

mkdir -p "$BUILD_CERES"
pushd "$BUILD_CERES"
cmake ../ceres-solver "${COMMON_CMAKE_ARGS[@]}" \
    "-DEIGEN_INCLUDE_DIR_HINTS=$EIGEN_DIR" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DEXPORT_BUILD_DIR=ON \
    -DOPENMP=ON
make -j$NUM_PARALLEL_BUILDS ceres
popd



BUILD_PANGOLIN=thirdparty/build-pangolin
rm -rf "$BUILD_PANGOLIN"

mkdir -p "$BUILD_PANGOLIN"
pushd "$BUILD_PANGOLIN"
cmake ../Pangolin
make -j$NUM_PARALLEL_BUILDS
popd
