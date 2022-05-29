BUILD_OPTIONS="-lgmp -lmpfr -I../geometry/include -I../geometry/test -std=c++20 -O3"

g++ main.cpp $BUILD_OPTIONS -o main
