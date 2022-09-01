import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "protos", "generated")) # Path to compiled proto files
import time_step_pb2
from google.protobuf.internal.decoder import _DecodeVarint32

import pandas as pd

class SimData:
    """Wrapper class for parsing protobuf experiment data."""

    def __init__(self, log_data_path, commands_path=''):
        """Constructor"""

        # Data dict
        self.data = {}

        self.read_log_data(log_data_path)

        # Add user comamnds if csv file exists
        if commands_path:
            self.read_commands(commands_path)


    def read_log_data(self, log_data_path):
        """Parses a protobuf file with simulation logs."""

        with open(log_data_path, 'rb') as f:
            buf = f.read()
            n = 0

            # Read each timestep
            while n < len(buf):
                msg_len, new_pos = _DecodeVarint32(buf, n)
                n = new_pos
                msg_buf = buf[n:n+msg_len]
                n += msg_len
                read_data = time_step_pb2.TimeStep()
                read_data.ParseFromString(msg_buf)

                # Create an entry for the timestep
                self.data[read_data.time] = {}
                self.data[read_data.time]['log'] = read_data
                self.data[read_data.time]['commands'] = [] # Create empty list of commands
                
            # Store experiment summary
            if self.data:

                # Total timesteps
                self.totalTime = len(self.data)

                # Points scored
                self.totalPoints = self.data[self.totalTime]['log'].points

                # Number of leaders and robots
                leaderNames = []
                teamNames = []
                for robot in self.data[1]['log'].robots:
                    if robot.state == time_step_pb2.Robot.LEADER:
                        leaderNames.append(robot.name)
                        teamNames.append(robot.teamID)

                self.leaders = leaderNames
                self.teams = teamNames
                self.numLeaders = len(self.leaders)
                self.numWorkers = len(self.data[1]['log'].robots) - self.numLeaders

                # Number of tasks that appeared in the experiment
                maxId = 0
                for task in self.data[self.totalTime]['log'].tasks:
                    id = int(task.name[5:])
                    if id > maxId:
                        maxId = id

                self.numTasks = maxId


    def read_commands(self, commands_path):
        """Parses a csv file with user commands."""

        df = pd.read_csv(commands_path)
        # print(df.to_string())

        self.users = set()

        # iterate row
            # append command (type, value, user, robot) to time

        for ind in df.index:
            command = {}

            time = df['TIME'][ind]
            command['time'] = time
            command['type'] = df['COMMAND'][ind]
            command['user'] = df['USER'][ind]
            command['value'] = df['VALUE'][ind]

            # Assume valid username is less than 8 characters
            if len(str(command['user'])) < 8:
                self.data[time]['commands'].append(command)
                self.users.add(int(command['user']))

            # print(self.data[time]['commands'])


    def __getitem__(self, key):
        return self.data[key]
    

if __name__ == "__main__":
    log_data_path = '/home/genki/GIT/argos-sct/results/SHSB211/results_2022-07-12_15-35/user_study_scenario1/user_study_scenario1_002/log_data.pb'
    # commands_path = '/home/genki/GIT/argos-sct/results/SHSB211/results_2022-07-12_15-35/user_study_scenario1/user_study_scenario1_002/commands.csv'
    # s = SimData(log_data_path, commands_path=commands_path)
    s = SimData(log_data_path)

    print('Total time: {}'.format(s.totalTime))
    print('Points: {}'.format(s.data[s.totalTime]['log'].points))
    print('Number of leaders: {}'.format(s.numLeaders))
    for leader in s.leaders:
        print('\t{}'.format(leader))
    print('Teams:')
    for team in s.teams:
        print('\t{}'.format(team))
    print('Number of followers: {}'.format(s.numWorkers))
    print('Number of tasks: {}'.format(s.numTasks))
    print(type(s.data[s.totalTime]))

    ########
    # TEST #
    ########

    # Calculate each team's average distance traveled
    import numpy as np

    # List of leader names = []
    teamNames = s.teams
    # Average team position = {}
    teamPosition = {}
    # Total distance traveled = {}
    distanceTraveled = {}
    for team in s.teams:
        distanceTraveled[team] = 0

    # for each timestep
        # for each team (loop by leader)
            # calculate the new average x and y of both teams (leader and followers)
            # if timestep != 1
                # calculate the distance from the previous team average to the new team average and add it to the sum
            # update previous team average

    for time in range(1, s.totalTime+1):

        # print('--------\ntime: {}'.format(time))

        # Add robot positions into separate arrays
        tempTeamPosition = {}
        for team in s.teams:
            tempTeamPosition[team] = []

        # Add robot positions according to their teams
        for robot in s.data[time]['log'].robots:
            for team in s.teams:
                if robot.teamID == team:
                    tempTeamPosition[team].append([robot.position.x, robot.position.y])

        newTeamPosition = {}

        # Calculate the average team positions
        for key, value in tempTeamPosition.items():
            newTeamPosition[key] = np.average(np.array(value), axis=0)
            # np.round(newTeamPosition[key], decimals=2, out=newTeamPosition[key])
            # print('team {0} new pos: {1}'.format(key, newTeamPosition[key]))

        if time != 1:
            for team in s.teams:
                # Calculate the distance traveled from the previous timestep
                x1 = teamPosition[team][0]
                y1 = teamPosition[team][1]
                x2 = newTeamPosition[team][0]
                y2 = newTeamPosition[team][1]
                distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
                distanceTraveled[team] += distance

                # print('team {0} dist: {1}'.format(team, distanceTraveled[team]))

        teamPosition = newTeamPosition


        # if time == 200:
        #     sys.exit(0)

    print('--------\nDistance traveled')
    for team in s.teams:
        print('team {0} dist: {1}'.format(team, distanceTraveled[team]))




    # Number of robots sent
    numRobotsSent = 0

    for time in range(1, s.totalTime+1):
        commands = s.data[time]['commands']
        for command in commands:
            if command['type'] == 'send':
                numRobotsSent += int(command['value'])
                print('sent {}'.format(command['value']))

    print('Number of robots sent: {}'.format(numRobotsSent))
        