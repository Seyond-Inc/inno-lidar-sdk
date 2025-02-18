#!/bin/bash

CURRENT_DIR=$(cd $(dirname $0); pwd)
PROJECT_DIR=$(cd $(dirname $CURRENT_DIR); pwd)

#qnx cross compile directory
if [ -z "$QNX_BASE" ]; then
  QNX_BASE=/opt/qos222/
fi
QNX_HOST=$QNX_BASE/host/linux/x86_64
QNX_TARGET=$QNX_BASE/target/qnx7
MAKEFLAGS=-I$QNX_BASE/target/qnx7/usr/include
PATH=$QNX_HOST/usr/bin:/bin:$PATH

export QNX_BASE QNX_TARGET QNX_HOST PATH

export QNX_TOOLCHAIN_TRIPLE="aarch64-unknown-nto-qnx7.1.0"
export QNX_TOOLCHAIN_PREFIX="${QNX_HOST}/usr/bin/${QNX_TOOLCHAIN_TRIPLE}-"

export CC=${QNX_TOOLCHAIN_PREFIX}gcc
export CXX=${QNX_TOOLCHAIN_PREFIX}g++
export STRIP=${QNX_TOOLCHAIN_PREFIX}strip
export AR=${QNX_TOOLCHAIN_PREFIX}ar
export OBJCOPY=${QNX_TOOLCHAIN_PREFIX}objcopy
export RANLIB=${QNX_TOOLCHAIN_PREFIX}ranlib
export READELF=${QNX_TOOLCHAIN_PREFIX}readelf

export ARCH_TAG=-qnx
echo QNX_HOST=$QNX_HOST
echo QNX_TARGET=$QNX_TARGET

cd ${PROJECT_DIR}
build_dir="build_unix"
if [[ ! -d ${build_dir} ]]; then
    mkdir ${build_dir}
else
    rm -rf ${build_dir}/*
fi
cd ${PROJECT_DIR}/${build_dir}

if [[ "${ARCH_TAG}" ]]; then
    cmake_param="-DARCH_TAG=${ARCH_TAG} ${cmake_param}"
fi

cmake ${cmake_param} ..
make clean && make -j1
make install

# build lidar_util
cd ${PROJECT_DIR}/apps/tools/lidar_util
build_util_dir="build_util_dir"
if [[ ! -d ${build_util_dir} ]]; then
    mkdir ${build_util_dir}
else
    rm -rf ${build_util_dir}/*
fi
cd ${PROJECT_DIR}/apps/tools/lidar_util/${build_util_dir}
cmake ${cmake_param} -DCUSTOMER_RELEASE=ON ..
make clean && make -j1
make install


cd ${PROJECT_DIR}/${build_dir}

if [[ ! -d ${build_util_dir} ]]; then
    mkdir ${build_util_dir}
else
    rm -rf ${build_util_dir}/*
fi

if [[ "$shared" == "1" ]]; then
    cmake_param="${cmake_param} -DMAKE_SHARED=1"
    cmake ${cmake_param} ..
    make clean && make -j1
fi
cd ${CURRENT_DIR}
