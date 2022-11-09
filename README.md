# ARGoS SCT Swap

## Installation

### Install ARGoS

Visit the ARGoS website to download and install ARGoS: https://www.argos-sim.info/

### Install Plugins

Install the necessary plugins found [here](https://gitlab.com/genki_miyauchi/argos-sct-plugins.git).

```
git clone https://gitlab.com/genki_miyauchi/argos-sct-plugins.git
cd argos-sct-plugins
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src
make
make install
```

Run ```argos3 -q all``` to see that the plugins (i.e. e-puck_leader, circle_task, rectangle_task) have succesfully been installed.

## Loading plugins

```
export ARGOS_PLUGIN_PATH=/home/genki/GIT/argos-sct/build/e-puck_leader:/home/genki/GIT/argos-sct/build/circle_task:/home/genki/GIT/argos-sct/build/rectangle_task:/home/genki/GIT/argos-sct/build/utility
```

Optionally, add the above line in .bashrc to avoid setting it in every terminal.

## Running the simulation

```
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ../src
make
cd ..
argos3 -c experiments/experiment_template.argos
```

## Developing

To modify the behavior of the leader and follower, edit ```leader.h```, ```leader.cpp```, ```follower.h```, ```follower.cpp``` in ```src/controllers/leader/``` and ```src/controllers/follower/```.

To modify the text that appears above the leader and follower, edit ```manualcontrol_qtuser_functions.h``` and ```manualcontrol_qtuser_functions.cpp``` in ```src/loop_functions/leaderfollower_loopfunctions/```.

For code that needs to be executed before, after or in every timestep, edit ```experiment_loop_functions.h``` and ```experiment_loop_functions.cpp``` in ```src/loop_functions/leaderfollower_loopfunctions/```

To modify the message structure, edit ```robot_message.h``` and ```robot_message.cpp``` in ```src/utility/```.

## Updates

The original program was created by Genki Miyauchi (ACSE, The University of Sheffield). This is limited to a homogenous robot system. The program have been modified to incorporate the heterogenous system
