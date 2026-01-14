set -e

cd /Users/benedikthoward/Projects/ulysses-gnss-radio/build/Debug/cmake/stm32cubemx
/opt/homebrew/bin/ccmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
