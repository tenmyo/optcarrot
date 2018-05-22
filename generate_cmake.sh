#!/bin/sh
set -eu

rm -rf build/
mkdir build
cd build
CC=clang CXX=clang++ cmake -GNinja -Wdeprecated --check-system-vars .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
