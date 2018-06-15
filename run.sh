#!/bin/sh
set -eu
clear
cmake --build build/
/usr/share/clang/run-clang-tidy.py -p ./build -fix -format -header-filter=.\*
#valgrind --leak-check=full --track-origins=yes --show-reachable=yes --suppressions=suppressions.txt --gen-suppressions=yes \
/usr/bin/time  ./build/bin/optcarrot_cpp
