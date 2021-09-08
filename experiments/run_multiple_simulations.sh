#!/bin/sh

PROJECT_DIR=/home/genki/GIT/argos-sct
SCENARIO_DIR=$PROJECT_DIR/experiments/scenario

cd $PROJECT_DIR

for robot_num in 20 30 40
do
    CONFIG=${robot_num}R_5T_5000D
    SCENARIO_DIR=$PROJECT_DIR/experiments/$CONFIG

    for scenario in 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20
    do
        argos3 -c $SCENARIO_DIR/${CONFIG}_${scenario}.argos
    done
done