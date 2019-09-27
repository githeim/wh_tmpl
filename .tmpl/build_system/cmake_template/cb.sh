#!/bin/bash
echo Configuration and Build
rm build -rf
mkdir build
cd build ; cmake .. -G Ninja && ninja -v -j14
#cd build ; cmake .. && make -j


