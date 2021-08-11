import os.path
import sys
import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pprint
from collections import defaultdict

# Debugging
pp = pprint.PrettyPrinter(indent=4)


def plot_stats(argv):

    print('Loading yaml ...')

    # Load yaml
    with open(argv[0], 'r') as file:
        try:
            data = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)

    print('Finished loading')

    plot_filename = 'robot-states.pdf'
    plot_robot_states(data,
                      title='Number of robots in each team',
                      x_label='Timestep',
                      y_label='Number of robots',
                      out_filename=plot_filename)
    
    plot_filename = 'task-demands.pdf'
    plot_task_demands(data,
                      title='Remaining task demands',
                      x_label='Timestep',
                      y_label='Task demand',
                      out_filename=plot_filename)

    plot_filename = 'trajectories.pdf'
    plot_trajectories(data,
                      title='Robot trajectories',
                      x_label='X(m)',
                      y_label='Y(m)',
                      out_filename=plot_filename)


def plot_robot_states(data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    y1 = []
    y2 = []
    y3 = []

    for timestep in data:

        # Counters
        num_followers_team1 = 0
        num_followers_team2 = 0
        num_connectors = 0

        for robot in data[timestep]['robots']:
            if robot['state'] == 'FOLLOWER':
                if robot['teamid'] == 1:
                    num_followers_team1 += 1
                elif robot['teamid'] == 2:
                    num_followers_team2 += 1
            elif robot['state'] == 'CONNECTOR':
                num_connectors += 1

        # Fill data for timestep 0
        if timestep == 'time_1':
            y1.append(num_followers_team1)
            y2.append(num_followers_team2)
            y3.append(num_connectors)

        y1.append(num_followers_team1)
        y2.append(num_followers_team2)
        y3.append(num_connectors)
        
    total_time = len(data)+1
    total_robots = y1[0]+y2[0]

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y1, label = "Follower, team1")
    plt.plot(x, y2, label = "Follower, team2")
    plt.plot(x, y3, label = "Connector")

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xticks(np.arange(0, total_time, 100))
    plt.yticks(np.arange(0, total_robots, 5))

    plt.legend()

    plt.savefig(out_filename)
    plt.close()


def plot_task_demands(data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    y = []

    for timestep in data:

        demand = 0

        for task in data[timestep]['tasks']:
            demand += task['demand']

        # Fill data for timestep 0
        if timestep == 'time_1':
            y.append(demand)
        
        y.append(demand)

    total_time = len(data)+1
    total_demand = y[0]

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y, label = "Demand")

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xticks(np.arange(0, total_time, 100))
    # plt.yticks(np.arange(0, total_demand, 25))

    plt.savefig(out_filename)
    plt.close()


def plot_trajectories(data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    # x1 = []
    # y1 = []
    # x2 = []
    # y2 = []
    x = defaultdict(list)
    y = defaultdict(list)

    for timestep in data:

        for robot in data[timestep]['robots']:
            # if robot['id'] == 'L1':
            #     x1.append(robot['pos']['x'])
            #     y1.append(robot['pos']['y'])
            # elif robot['id'] == 'L2':
            #     x2.append(robot['pos']['x'])
            #     y2.append(robot['pos']['y'])
            x[robot['id']].append(robot['pos']['x'])
            y[robot['id']].append(robot['pos']['y'])

    # plotting the lines
    # plt.plot(x1, y1, label = "L1")
    # plt.plot(x2, y2, label = "L2")
    ids = list(x.keys())
    for id in ids:
        plt.plot(x[id], y[id], label = id)

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.xticks(np.arange(-2, 3, 1))
    plt.yticks(np.arange(-2, 3, 1))
    
    # plt.legend()

    plt.savefig(out_filename)
    plt.close()


if __name__ == "__main__":
    # plot_stats(sys.argv[1:])
    path = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/experiment.yaml')
    plot_stats([path])
