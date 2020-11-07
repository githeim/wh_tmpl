#!/bin/bash
# prerequisite packages for windheim template

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
sudo apt-get install libgtest-dev -y
pushd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make -j12
sudo cp *.a /usr/lib
sudo mkdir -p /usr/local/lib/gtest
sudo ln -s /usr/lib/libgtest.a /usr/local/lib/gtest/libgtest.a
sudo ln -s /usr/lib/libgtest_main.a /usr/local/lib/gtest/libgtest_main.a
popd

