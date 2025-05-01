#!/bin/bash
# prerequisite packages for windheim template

if [[ $(lsb_release -cs) == "bionic" ]]; then
  echo "Ubuntu 18.x"
  curl -sL https://deb.nodesource.com/setup_14.x | sudo bash -
  sudo apt -y install nodejs
elif [[ $(lsb_release -cs) == "focal" ]]; then
  echo "Ubuntu 20.x"
fi

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

sudo apt-get install -y python3-urwid
pip3 install pyyaml                                             
sudo apt-get install -y libgtest-dev
