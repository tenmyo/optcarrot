#!/bin/sh
set -eu
clear
cmake --build build/
/usr/share/clang/run-clang-tidy.py -p ./build -fix -format
#valgrind --leak-check=full --track-origins=yes --show-reachable=yes --suppressions=suppressions.txt --gen-suppressions=yes \
  ./build/bin/optcarrot_cpp
