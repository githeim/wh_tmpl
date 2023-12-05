simple Speech To Text example
=========================

# Overview
Introducing a simple speech to text example.
This program runs locally and does not use any network bandwidth.
## Software configuraton 
The program consists of a frontend and a backend.

The frontend is implemented in python. The frontend recognizes the voice and sends the recognized text to the posix mqueue. Voice recognition was implemented using the VOSK API(https://github.com/alphacep/vosk-api).

The backend is implemented in C++. The backend receives the recognized text through the posix mqueue and outputs it to a semi-transparent prompt implemented as Overlay. The semi-transparent overlay is implemented using SDL2.

# How to Build
```
$ cd stt_ex
$ ./cb.sh  #cb - configuration and build
```

# How to run and stop the application

Run the application
```
$ ./r.sh
```

Stop the application
```
$ ./stop.sh
```


