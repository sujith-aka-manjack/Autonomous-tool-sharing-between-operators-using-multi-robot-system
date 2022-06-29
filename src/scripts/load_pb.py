import os
import sys
 
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "protos", "generated"))
import time_step_pb2

from google.protobuf.internal.decoder import _DecodeVarint32


class SimData:
    """Wrapper class for parsing protobuf experiment data."""

    def __init__(self, path):
        """Constructor parses a protobuf file."""

        self.data = {}

        with open(path, 'rb') as f:
            buf = f.read()
            n = 0
            while n < len(buf):
                msg_len, new_pos = _DecodeVarint32(buf, n)
                n = new_pos
                msg_buf = buf[n:n+msg_len]
                n += msg_len
                read_data = time_step_pb2.TimeStep()
                read_data.ParseFromString(msg_buf)
                self.addItem(read_data)
                self.totalTime = read_data.time


    def addItem(self, timeStep):
        """Stores a single time step."""

        if not self.data:

            # Store meta data
            
            numLeaders = 0
            demand = 0

            for robot in timeStep.robots:
                if robot.state == time_step_pb2.Robot.LEADER:
                    numLeaders += 1
            
            for task in timeStep.tasks:
                demand += task.demand

            self.numLeaders = numLeaders
            self.numWorkers = len(timeStep.robots) - self.numLeaders
            self.numTasks = len(timeStep.tasks)
            self.totalDemand = demand

        self.data[timeStep.time] = timeStep


    def __getitem__(self, key):
        return self.data[key]


def main():

    path = '/home/genki/GIT/argos-sct/results/results_2022-06-24_16-11/user_study_scenario1/user_study_scenario1_001/log_data.pb'
    s = SimData(path)

    print(s.totalTime)
    print(s.numLeaders)
    print(s.numWorkers)
    print(s.numTasks)
    print(s.totalDemand)
    # print(s.data)

if __name__ == "__main__":
    main()