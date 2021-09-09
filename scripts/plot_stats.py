import os.path
import sys
from numpy.ma.extras import average
import yaml
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pprint
from collections import defaultdict

# Debugging
pp = pprint.PrettyPrinter(indent=4)

# Experiment Configurations
TOTAL_TIME = 4001
TOTAL_ROBOTS = 30
DEMAND_PER_TASK = 5000
NUMBER_OF_TASKS = 5
TOTAL_DEMAND = DEMAND_PER_TASK * NUMBER_OF_TASKS

RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/{0}R_{1}T_{2}D'.format(TOTAL_ROBOTS, NUMBER_OF_TASKS, DEMAND_PER_TASK))
OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/{0}R_{1}T_{2}D'.format(TOTAL_ROBOTS, NUMBER_OF_TASKS, DEMAND_PER_TASK))

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/40R_{}T_{}D'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK))
# OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/40R_{}T_{}D'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK))

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/6T_500D')
# OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/6T_500D')

def load_stats(argv):

    print('Loading yaml ...')
    start_time = time.time()

    stats = {}

    for file in argv:
        scenario = os.path.splitext(file)[0]
        path = os.path.join(RESULTS_DIR, file)

        start_time_single = time.time()

        # Load yaml
        with open(path, 'r') as file:
            try:
                data = yaml.load(file, Loader=yaml.CLoader)
                stats[scenario] = data
                duration_single = round(time.time() - start_time_single, 3)
                duration_total = round(time.time() - start_time, 3)
                print('Loaded {0} in {1} s ({2} s)'.format(scenario, duration_single, duration_total))
            except yaml.YAMLError as exc:
                print(exc)

    duration = round(time.time() - start_time, 3)
    print('Finished loading in {0} seconds'.format(duration))

    return stats


def plot_robot_states(scenario, data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    y1 = []
    y2 = []
    y3 = []
    y4 = []

    for timestep in data:

        # Counters
        num_followers_team1 = 0
        num_followers_team2 = 0
        num_connectors = 0
        num_travelers = 0

        for robot in data[timestep]['robots']:
            if robot['state'] == 'FOLLOWER':
                if robot['teamid'] == 1:
                    num_followers_team1 += 1
                elif robot['teamid'] == 2:
                    num_followers_team2 += 1
            elif robot['state'] == 'CONNECTOR':
                num_connectors += 1
            elif robot['state'] == 'TRAVELER':
                    num_travelers += 1

        # Fill data for timestep 0
        if timestep == 'time_1':
            y1.append(num_followers_team1)
            y2.append(num_followers_team2)
            y3.append(num_connectors)
            y4.append(num_travelers)

        y1.append(num_followers_team1)
        y2.append(num_followers_team2)
        y3.append(num_connectors)
        y4.append(num_travelers)
        
    total_time = len(data)+1
    total_robots = y1[0]+y2[0]

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y1, label = "Follower, team1")
    plt.plot(x, y2, label = "Follower, team2")
    plt.plot(x, y3, label = "Connector")
    if np.amax(y4) > 0:
        plt.plot(x, y4, label = "Traveler")

    # plt.title(title)
    plt.xlabel(x_label, fontsize=20)
    plt.ylabel(y_label, fontsize=20)

    plt.xticks(np.arange(0, total_time, 250), fontsize=15)
    plt.yticks(np.arange(0, total_robots, 5), fontsize=15)

    plt.ylim([0,y_limit])

    plt.legend()

    plt.savefig(out_filename)
    plt.close()


def plot_overall_robot_states(stats, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    # total_time = len(next(iter(stats)))+1
    total_time = TOTAL_TIME

    y1_total = []
    y2_total = []
    y3_total = []
    y4_total = []

    for scenario, data in stats.items():

        y1 = []
        y2 = []
        y3 = []
        y4 = []

        for timestep in data:

            # Counters
            num_followers_team1 = 0
            num_followers_team2 = 0
            num_connectors = 0
            num_travelers = 0

            for robot in data[timestep]['robots']:
                if robot['state'] == 'FOLLOWER':
                    if robot['teamid'] == 1:
                        num_followers_team1 += 1
                    elif robot['teamid'] == 2:
                        num_followers_team2 += 1
                elif robot['state'] == 'CONNECTOR':
                    num_connectors += 1
                elif robot['state'] == 'TRAVELER':
                    num_travelers += 1

            # Fill data for timestep 0
            if timestep == 'time_1':
                y1.append(num_followers_team1)
                y2.append(num_followers_team2)
                y3.append(num_connectors)
                y4.append(num_travelers)

            y1.append(num_followers_team1)
            y2.append(num_followers_team2)
            y3.append(num_connectors)
            y4.append(num_travelers)

        y1_total.append(y1)
        y2_total.append(y2)
        y3_total.append(y3)
        y4_total.append(y4)

    y1_total = np.array(y1_total)
    y2_total = np.array(y2_total)
    y3_total = np.array(y3_total)
    y4_total = np.array(y4_total)

    print(y1_total.shape)
    print(y1_total)

    y1_mean = np.mean(y1_total, axis=0)
    y2_mean = np.mean(y2_total, axis=0)
    y3_mean = np.mean(y3_total, axis=0)
    y4_mean = np.mean(y4_total, axis=0)

    y1_min = np.min(y1_total, axis=0)
    y2_min = np.min(y2_total, axis=0)
    y3_min = np.min(y3_total, axis=0)
    y4_min = np.min(y4_total, axis=0)

    y1_max = np.max(y1_total, axis=0)
    y2_max = np.max(y2_total, axis=0)
    y3_max = np.max(y3_total, axis=0)
    y4_max = np.max(y4_total, axis=0)

    y1_5 = np.percentile(y1_total, 5, axis=0)
    y2_5 = np.percentile(y2_total, 5, axis=0)
    y3_5 = np.percentile(y3_total, 5, axis=0)
    y4_5 = np.percentile(y4_total, 5, axis=0)

    y1_95 = np.percentile(y1_total, 95, axis=0)
    y2_95 = np.percentile(y2_total, 95, axis=0)
    y3_95 = np.percentile(y3_total, 95, axis=0)
    y4_95 = np.percentile(y4_total, 95, axis=0)

    # print(y1_mean.shape)
    # print(y1_mean)

    # total_robots = y1[0]+y2[0]
    total_robots = TOTAL_ROBOTS

    x = range(0,total_time)
    x = [elem / 10 for elem in x]

    # plotting the lines
    plt.plot(x, y1_mean, label = "Follower 1")
    plt.plot(x, y2_mean, label = "Follower 2")
    plt.plot(x, y3_mean, label = "Connector")
    if np.amax(y4_mean) > 0:
        plt.plot(x, y4_mean, label = "Traveler")

    # plt.title(title)
    plt.xlabel(x_label, fontsize=15)
    plt.ylabel(y_label, fontsize=15)

    plt.xticks(np.arange(0, total_time/10, 50), fontsize=12)
    # plt.yticks(np.arange(0, total_robots, 5), fontsize=12)
    plt.yticks(np.arange(0, 25, 5), fontsize=12)

    plt.ylim([0,y_limit])

    plt.legend(loc='upper left')

    plt.savefig('{}{}.pdf'.format(os.path.splitext(out_filename)[0], '_mean'))

    plt.fill_between(x, y1_min, y1_max, alpha=0.2)
    plt.fill_between(x, y2_min, y2_max, alpha=0.2)
    plt.fill_between(x, y3_min, y3_max, alpha=0.2)
    if np.amax(y4_max) > 0:
        plt.fill_between(x, y4_min, y4_max, alpha=0.2)

    # plt.fill_between(x, y1_5, y1_95, alpha=0.2)
    # plt.fill_between(x, y2_5, y2_95, alpha=0.2)
    # plt.fill_between(x, y3_5, y3_95, alpha=0.2)
    # if np.amax(y4_95) > 0:
    #     plt.fill_between(x, y4_5, y4_95, alpha=0.2)

    plt.savefig(out_filename)
    plt.close()


def plot_task_demands(scenario, data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
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

    # plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xticks(np.arange(0, total_time, 500))
    # plt.yticks(np.arange(0, total_demand, 25))

    plt.ylim([0,y_limit])

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
    total_time = TOTAL_TIME
    # total_demand = y[0]
    total_demand = TOTAL_DEMAND

    x = range(0,total_time)

    # plotting the lines
    plt.plot(x, y_mean, label = "Demand")
    plt.fill_between(x, y_min, y_max, alpha=0.2)

    # plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xticks(np.arange(0, total_time, 250))
    # plt.yticks(np.arange(0, total_demand, 25))

    plt.ylim([0,y_limit])

    plt.savefig(out_filename)
    plt.close()


def average_completion_time(stats):

    for scenario, data in stats.items():

        for timestep in data:

            demand = 0

            for task in data[timestep]['tasks']:
                demand += task['demand']

            if demand == 0:
                print('Completed at {}'.format(timestep))
                return


def plot_trajectories(scenario, data, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
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

    # plt.title(title)
    plt.xlabel(x_label, fontsize=20)
    plt.ylabel(y_label, fontsize=20)

    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.xticks(np.arange(-2, 3, 1))
    plt.yticks(np.arange(-2, 3, 1))
    
    # plt.legend()

    plt.savefig(out_filename)
    plt.close()


def plot_overall_connected_ratio(stats, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    ratios = {}

    for scenario in stats:
        num_robots = scenario.split('_')[0][:-1]
        if num_robots not in ratios:
            ratios[num_robots] = []
            print(num_robots)

    # ratios = []

    # For each experiment
    for scenario, data in stats.items():

        print('Scenario: {0}'.format(scenario))
        num_robots = scenario.split('_')[0][:-1]

        L1_sent = []
        L1_received = []
        L2_sent = []
        L2_received = []

        L1_total = []

        for timestep in data:

            # Get the number of unique sent and received messages of L1 and L2
            for robot in data[timestep]['robots']:
                if robot['state'] == 'LEADER':
                    if robot['id'] == 'L1':
                        L1_sent.append(robot['beat_sent'])
                        L1_received.append(robot['beat_received'])
                    elif robot['id'] == 'L2':
                        L2_sent.append(robot['beat_sent'])
                        L2_received.append(robot['beat_received'])
                
        # Convert lists into sets
        L1_sent = set(L1_sent)
        L1_received = set(L1_received)
        L2_sent = set(L2_sent)
        L2_received = set(L2_received)
       
        # Sum the sent
        sum_sent = len(L1_sent) + len(L2_sent)
        print('Sent: {0}'.format(sum_sent))

        # Sum the received
        sum_received = len(L1_received) + len(L2_received)
        print('Received: {0}'.format(sum_received))

        # Calculate ratio
        ratio = sum_received / sum_sent

        ratios[num_robots].append(ratio)

    # Average the ratio
    # print('Mean: {0}'.format(np.mean(ratios)))

    # average_ratios = []
    # average_ratios.append(np.mean(ratios))

    # Plot
    # x = [20]
    # plt.bar(x, average_ratios, tick_label=x)

    # Draw dashed line
    plt.axhline(y=1, linewidth=1, color='black', linestyle='--')

    # for i, v in enumerate(average_ratios):
    #     plt.text(x[i] - 0.1, v - 0.05, str(v), color='white')

    # data_1 = ratios['20']
    data_2 = ratios['30']
    # data_3 = ratios['40']

    data = [data_2]
    # data = [data_1, data_2]
    # data = [data_1, data_2, data_3]
    
    plt.boxplot(data)

    # plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.xticks(range(1,len(ratios)+1), ratios.keys())
    plt.yticks(np.arange(0, 1.1, 0.2))

    plt.savefig(out_filename)
    plt.close()


def main(argv):
    stats = load_stats(argv)

    # Plot single stats
    for scenario, data in stats.items():

        plot_filename = '{0}/robot-states_{1}.pdf'.format(OUTPUT_DIR, scenario)
        plot_robot_states(scenario,
                          data,
                          title='Number of robots in each team',
                          x_label='Time (seconds)',
                          y_label='Number of robots',
                          out_filename=plot_filename)
        
        plot_filename = '{0}/task-demands_{1}.pdf'.format(OUTPUT_DIR, scenario)
        plot_task_demands(scenario,
                          data,
                          title='Remaining task demands',
                          x_label='Time (seconds)',
                          y_label='Task demand',
                          out_filename=plot_filename)

        # plot_filename = '{0}/trajectories_{1}.pdf'.format(OUTPUT_DIR, scenario)
        # plot_trajectories(scenario,
        #                   data,
        #                   title='Robot trajectories',
        #                   x_label='X(m)',
        #                   y_label='Y(m)',
        #                   out_filename=plot_filename)

    average_completion_time(stats)

    # Plot overall stats
    plot_filename = '{0}/overall_robot-states_{1}.pdf'.format(OUTPUT_DIR, next(iter(stats))[:-3])
    plot_overall_robot_states(stats,
                              title='Average number of robots in each team',
                              x_label='Time (seconds)',
                              y_label='Number of robots',
                              out_filename=plot_filename)

    plot_filename = '{0}/overall_task_demands_{1}.pdf'.format(OUTPUT_DIR, next(iter(stats))[:-3])
    plot_overall_task_demands(stats,
                              title='Average remaining task demand',
                              x_label='Timestep (seconds)',
                              y_label='Task demand',
                              out_filename=plot_filename)


    # Run connected ratio separately from the other plotting functions for now as it deals with scenarios with different robot numbers

    plot_filename = '{0}/overall_connected_ratio.pdf'.format(OUTPUT_DIR)
    plot_overall_connected_ratio(stats,
                                 title='Average ratio of beat messages exchanged',
                                 x_label='Number of follower robots',
                                 y_label='Ratio of messages received',
                                 out_filename=plot_filename)


if __name__ == "__main__":
    # plot_stats(sys.argv[1:])
    # argv = [
    #         '20R_6T_100D_01.yaml', 
    #         '20R_6T_100D_02.yaml'
    #        ]
    argv = []

    # for i in range(1,21):
    #     id = ''
    #     if(i < 10):
    #         id += '0' + str(i)
    #     else:
    #         id += str(i)
    #     argv.append('20R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    for i in range(1,21):
        id = ''
        if(i < 10):
            id += '0' + str(i)
        else:
            id += str(i)
        argv.append('30R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    # for i in range(1,21):
    #     id = ''
    #     if(i < 10):
    #         id += '0' + str(i)
    #     else:
    #         id += str(i)
    #     argv.append('40R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    main(argv)
