#!/bin/sh
set -e

if [ ! -d "$OPENCV_INSTALL_DIR/lib" ]; then
  wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.tar.gz -O opencv-$OPENCV_VERSION.tar.gz
  tar xzf opencv-$OPENCV_VERSION.tar.gz
  rm opencv-$OPENCV_VERSION.tar.gz
  cd opencv-$OPENCV_VERSION
  mkdir build
  cd build

  cmake -DWITH_CUDA=OFF -DENABLE_AVX=ON -DWITH_OPENGL=ON -DWITH_TBB=ON -DBUILD_opencv_apps=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX="$OPENCV_INSTALL_DIR" -DPYTHON3_EXECUTABLE=$(which python3) -DPYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") ..
  make -j2
  make install
else
  echo "Using cached opencv3 install."
fi
