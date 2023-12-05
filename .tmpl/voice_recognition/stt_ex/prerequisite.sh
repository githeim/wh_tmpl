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

sudo apt-get install -y ninja-build cmake
pip3 install sounddevice
sudo apt-get install libportaudio2
pip3 install vosk
pip3 install ipcqueue
sudo apt-get install -y libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev

