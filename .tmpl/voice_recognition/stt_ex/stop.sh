#!/bin/bash

echo stop the frontend and backend applications
pkill -2 -f python3.*frontend
pkill stt_ex*
