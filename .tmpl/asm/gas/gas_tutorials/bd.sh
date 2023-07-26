#!/bin/bash
set -e
source ./common.sh
echo build GAS\(GNU Assembler\) examples

only_asm_build() {
  echo build example [$1]
  gcc -c $1.s && gcc -static $1.o -o $1
}

build_with_c_caller() {
  echo build example [call$1]
  gcc  call$1.c $1.s -o call$1
}

# Build hello world
echo build example [hello]
gcc -c hello.s && ld hello.o -o hello

for VAR in  $EXAMPLE_LIST
do
  only_asm_build $VAR
done

# the examples that are mixed with the c language caller
# they build with c code
for VAR in $EXAMPLE_WITH_C_CALLER
do
  build_with_c_caller $VAR
done
