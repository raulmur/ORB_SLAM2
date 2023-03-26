#!/usr/bin/env bash

set -e

find ./include ./src  -iname "*.hpp" -or -iname "*.h" -or -iname "*.cpp" -or -iname "*.cc" | xargs clang-format -i --verbose
