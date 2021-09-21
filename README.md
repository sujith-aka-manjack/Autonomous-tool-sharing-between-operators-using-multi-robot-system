# ARGoS SCT

## Installation

### Install ARGoS

Visit the ARGoS website to download and install ARGoS: https://www.argos-sim.info/

### Install Dependencies

```
pip install -r requirements.txt --no-dependencies
```

The code has been developed and tested in Python 3.9.5.

## Load custom ARGoS plugins

```
export ARGOS_PLUGIN_PATH=/home/genki/GIT/argos-sct/build/e-puck_leader:/home/genki/GIT/argos-sct/build/circle_task:/home/genki/GIT/argos-sct/build/rectangle_task:/home/genki/GIT/argos-sct/build/utility
```

Optionally, add the above line in .bashrc to avoid setting it in every terminal.

## Running the simulation

```
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..
argos3 -c experiments/experiment_template.argos
```