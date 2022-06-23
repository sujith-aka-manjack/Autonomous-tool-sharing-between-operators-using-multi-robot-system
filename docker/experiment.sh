#!/bin/bash

# Script to run commands during the webviz user study.
# 
# 1) To run the docker container with the ARGoS simulation:
#
#   $ ./experiment.sh run
#
# 2) To compress the result data:
#
#   $ ./experiment.sh compress
#

mode=$1

echo "Mode: $mode"

if [ $mode = "run" ]
then
    echo "Starting docker container..."
    cd ~/Documents/experiment
    docker run --name argos -d -p 3000:3000 -p 8000:8000 -p 5000:5000 -v $(pwd)/results:/home/docker/argos-sct/results -w /home/docker/argos-sct genki15/argos-sct python3 src/web_app/app.py
elif [ $mode = "compress" ]
then
    echo "Compressing experiment data..."
    cd ~/Documents/experiment
    filename=$(date +"results_%Y-%m-%d_%H:%M")
    zip -r ./$filename.zip results/*
    mv results $filename
else
    echo "Unrecognised mode"
fi
