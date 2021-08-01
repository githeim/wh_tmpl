#!/bin/bash
EXE_FILE=ping_checker-121212.343456.out

# :x: by using 'setcap' command user can send the ping without sudo
# :x: this command is needed only one time
sudo setcap cap_net_raw,cap_net_admin=eip build/Debug/$EXE_FILE 
cd build/Debug ;  ./$EXE_FILE 127.0.0.1
