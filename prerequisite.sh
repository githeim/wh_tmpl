#!/bin/bash
# prerequisite packages for windheim template

sudo apt-get update
sudo apt-get install -y build-essential                             
sudo apt-get install -y cmake                                       
sudo apt-get install -y python python-dev python3                   
sudo apt-get install -y python3-pip                                 
sudo apt-get install -y libncurses5-dev                             
sudo apt-get install -y unzip git zip                               
sudo apt-get install -y make                                        
sudo apt-get install -y wget tmux curl                              
sudo apt-get install -y g++ lcov doxygen graphviz rpcbind           
sudo apt-get install -y gcc-arm-linux-gnueabi g++-arm-linux-gnueabi 
sudo apt-get install -y build-essential                             
sudo apt-get install -y ninja-build                                 
                                                                
pip3 install urwid                                              
pip3 install pyyaml                                             

# google test installation
mkdir -p gtest_build
pushd gtest_build
git clone https://github.com/google/googletest.git
popd
mkdir -p gtest_build/googletest/build

pushd gtest_build/googletest/build
cmake ..
make -j`nproc`
sudo make install 
popd
