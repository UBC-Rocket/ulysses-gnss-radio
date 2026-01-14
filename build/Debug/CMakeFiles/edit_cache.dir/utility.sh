set -e

cd /Users/benedikthoward/Projects/ulysses-gnss-radio/build/Debug
/opt/homebrew/bin/ccmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
