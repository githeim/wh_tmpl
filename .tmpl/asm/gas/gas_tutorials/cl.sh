#!/bin/bash

source common.sh
echo Clean up files

rm hello.o hello
#EXAMPLE_LIST='hello callmaxofthree hola fib echo power average callsum callfactorial'
for VAR in $EXAMPLE_LIST
do
  echo Delete $VAR.o and $VAR
  rm $VAR.o $VAR
done

for VAR in $EXAMPLE_WITH_C_CALLER
do
  echo Delete call$VAR
  rm call$VAR
done
