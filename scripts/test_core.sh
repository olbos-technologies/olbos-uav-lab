#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror -fsanitize=undefined -Icore/include tests/unit/core_test.cpp -o build/core_test
./build/core_test
