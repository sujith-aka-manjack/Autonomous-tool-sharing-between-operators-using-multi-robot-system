# ARGoS SCT

## Running the simulation

```
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
cd ..
argos3 -c experiments/leader_follower.argos
```