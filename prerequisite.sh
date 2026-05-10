#!/bin/bash
error() {
  local parent_lineno="$1"
  local message="$2"
  local code="${3:-1}"
  if [[ -n "$message" ]] ; then
    echo "Error on or near line ${parent_lineno}: ${message}; exiting with status ${code}"
  else
    echo "Error on or near line ${parent_lineno}; exiting with status ${code}"
  fi
  exit "${code}"
}
trap 'error ${LINENO}' ERR

# prerequisite packages for windheim template

sudo apt-get update
sudo apt-get install -y build-essential                             
sudo apt-get install -y cmake                                       
sudo apt-get install -y python3                   
sudo apt-get install -y python3-pip                                 
sudo apt-get install -y libncurses5-dev                             
sudo apt-get install -y unzip git zip                               
sudo apt-get install -y make                                        
sudo apt-get install -y wget tmux curl                              
sudo apt-get install -y g++ lcov doxygen graphviz rpcbind           
sudo apt-get install -y gcc-arm-linux-gnueabi g++-arm-linux-gnueabi 
sudo apt-get install -y build-essential                             
sudo apt-get install -y ninja-build                                 

pip3 install --break-system-packages pyyaml
pip3 install --break-system-packages textual
sudo apt-get install -y libgtest-dev
