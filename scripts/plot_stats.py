import os.path
import sys
import yaml
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pprint
from collections import defaultdict

# Debugging
pp = pprint.PrettyPrinter(indent=4)

RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results')
OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/summary')


def load_stats(argv):

    print('Loading yaml ...')
    start_time = time.time()

    stats = {}

    for file in argv:
        scenario = os.path.splitext(file)[0]
        path = os.path.join(RESULTS_DIR, file)

        # Load yaml
        with open(path, 'r') as file:
            try:
                data = yaml.load(file, Loader=yaml.CLoader)
                stats[scenario] = data
                print('Loaded {0}'.format(scenario))
            except yaml.YAMLError as exc:
                print(exc)

    duration = round(time.time() - start_time, 3)
    print('Finished loading in {0} seconds'.format(duration))

    return stats


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


def plot_overall_robot_states(stats, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    # total_time = len(next(iter(stats)))+1
    total_time = 2001

    y1_total = []
    y2_total = []
    y3_total = []

    for scenario, data in stats.items():

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

        y1_total.append(y1)
        y2_total.append(y2)
        y3_total.append(y3)

    y1_total = np.array(y1_total)
    y2_total = np.array(y2_total)
    y3_total = np.array(y3_total)

    # print(y1_total.shape)
    # print(y1_total)

    y1_mean = np.mean(y1_total, axis=0)
    y2_mean = np.mean(y2_total, axis=0)
    y3_mean = np.mean(y3_total, axis=0)

    y1_min = np.min(y1_total, axis=0)
    y2_min = np.min(y2_total, axis=0)
    y3_min = np.min(y3_total, axis=0)
    
    y1_max = np.max(y1_total, axis=0)
    y2_max = np.max(y2_total, axis=0)
    y3_max = np.max(y3_total, axis=0)

    # print(y1_mean.shape)
    # print(y1_mean)

    # total_robots = y1[0]+y2[0]
    total_robots = 40

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y1_mean, label = "Follower, team1")
    plt.plot(x, y2_mean, label = "Follower, team2")
    plt.plot(x, y3_mean, label = "Connector")
    plt.fill_between(x, y1_min, y1_max, alpha=0.2)
    plt.fill_between(x, y2_min, y2_max, alpha=0.2)
    plt.fill_between(x, y3_min, y3_max, alpha=0.2)

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


def plot_overall_task_demands(stats, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    y_total = []

    for scenario, data in stats.items():

        y = []

        for timestep in data:

            demand = 0

            for task in data[timestep]['tasks']:
                demand += task['demand']

            # Fill data for timestep 0
            if timestep == 'time_1':
                y.append(demand)
            
            y.append(demand)

        y_total.append(y)

    y_total = np.array(y_total)

    y_mean = np.mean(y_total, axis=0)
    y_min = np.min(y_total, axis=0)
    y_max = np.max(y_total, axis=0)

    # total_time = len(data)+1
    total_time = 2001
    # total_demand = y[0]
    total_demand = 600

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y_mean, label = "Demand")
    plt.fill_between(x, y_min, y_max, alpha=0.2)

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


def main(argv):
    stats = load_stats(argv)

    # Plot single stats
    for scenario, data in stats.items():

        plot_filename = '{0}/robot-states_{1}.pdf'.format(OUTPUT_DIR, scenario)
        plot_robot_states(data,
                          title='Number of robots in each team',
                          x_label='Timestep',
                          y_label='Number of robots',
                          out_filename=plot_filename)
        
        plot_filename = '{0}/task-demands_{1}.pdf'.format(OUTPUT_DIR, scenario)
        plot_task_demands(data,
                          title='Remaining task demands',
                          x_label='Timestep',
                          y_label='Task demand',
                          out_filename=plot_filename)

        plot_filename = '{0}/trajectories_{1}.pdf'.format(OUTPUT_DIR, scenario)
        plot_trajectories(data,
                          title='Robot trajectories',
                          x_label='X(m)',
                          y_label='Y(m)',
                          out_filename=plot_filename)

    # Plot overall stats
    plot_filename = '{0}/overall_robot-states_{1}.pdf'.format(OUTPUT_DIR, next(iter(stats))[:-3])
    plot_overall_robot_states(stats,
                              title='Average number of robots in each time',
                              x_label='Timestep',
                              y_label='Number of robots',
                              out_filename=plot_filename)

    plot_filename = '{0}/overall_task_demands_{1}.pdf'.format(OUTPUT_DIR, next(iter(stats))[:-3])
    plot_overall_task_demands(stats,
                              title='Average remaining task demand',
                              x_label='Timestep',
                              y_label='Task demand',
                              out_filename=plot_filename)




if __name__ == "__main__":
    # plot_stats(sys.argv[1:])
    # argv = [
    #         '20R_6T_100D_01.yaml', 
    #         '20R_6T_100D_02.yaml'
    #        ]
    argv = []
    for i in range(1,20):
        id = ''
        if(i < 10):
            id += '0' + str(i)
        else:
            id += str(i)
        argv.append('40R_6T_100D_{0}.yaml'.format(id))

    main(argv)
