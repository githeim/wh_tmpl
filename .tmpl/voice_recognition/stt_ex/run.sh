#!/bin/bash

echo Start STT frontend and backend
pushd backend
./r.sh &
popd

pushd frontend
./r.sh &
popd

