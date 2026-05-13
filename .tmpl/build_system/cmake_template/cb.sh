#!/bin/bash
echo Configuration and Build
[ -f compile_commands.json ] && rm compile_commands.json
[ -d build ] && rm -rf build
mkdir -p build
cd build ; cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -G Ninja \
  && ln -s build/compile_commands.json .. \
  && ninja -v -j14
#cd build ; cmake .. && make -j


