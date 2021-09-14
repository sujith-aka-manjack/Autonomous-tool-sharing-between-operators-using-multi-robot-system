import os.path
import sys
from numpy.ma.extras import average
import yaml
import time
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import pprint
from collections import defaultdict

# Debugging
pp = pprint.PrettyPrinter(indent=4)

# Experiment Configurations
TOTAL_ROBOTS = 30
DEMAND_PER_TASK = 5000
NUMBER_OF_TASKS = 5
TOTAL_DEMAND = DEMAND_PER_TASK * NUMBER_OF_TASKS

RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/{0}R_{1}T_{2}D'.format(TOTAL_ROBOTS, NUMBER_OF_TASKS, DEMAND_PER_TASK))
# OUTPUT_DIR = RESULTS_DIR

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/40R_{}T_{}D'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK))
# OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/40R_{}T_{}D'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK))

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/6T_500D')
# OUTPUT_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/6T_500D')

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/{0}R_{1}T_{2}D_no_exchange'.format(TOTAL_ROBOTS, NUMBER_OF_TASKS, DEMAND_PER_TASK))
# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results/{0}R_{1}T_{2}D_exchange'.format(TOTAL_ROBOTS, NUMBER_OF_TASKS, DEMAND_PER_TASK))
OUTPUT_DIR = RESULTS_DIR

# RESULTS_DIR = os.path.join(os.environ['HOME'], 'GIT/argos-sct/results')
# OUTPUT_DIR = RESULTS_DIR


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
    plt.plot(x, y1, label = "Team 1 Follower")
    plt.plot(x, y2, label = "Team 2 Follower")
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
    # total_time = TOTAL_TIME
    longest_sim_time = 0

    y1_total = []
    y2_total = []
    y3_total = []
    y4_total = []

    for scenario, data in stats.items():

        if len(data)+1 > longest_sim_time:
            longest_sim_time = len(data)+1

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

    total_time = longest_sim_time

    # Fill empty timesteps with last value
    for states in [y1_total, y2_total, y3_total, y4_total]:
        for y in states:
            if(len(y) < total_time):
                # print('SHORT')
                last_val = y[-1]
                for i in range(0,total_time-len(y)):
                    y.append(last_val)
            # else:
            #     print('LONGEST')    

    # print(len(y1_total[0]))
    # print(len(y1_total[1]))
    # print(len(y2_total[0]))
    # print(len(y2_total[1]))
    # print(len(y3_total[0]))
    # print(len(y3_total[1]))
    # print(len(y4_total[0]))
    # print(len(y4_total[1]))

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

    # y1_5 = np.percentile(y1_total, 5, axis=0)
    # y2_5 = np.percentile(y2_total, 5, axis=0)
    # y3_5 = np.percentile(y3_total, 5, axis=0)
    # y4_5 = np.percentile(y4_total, 5, axis=0)

    # y1_95 = np.percentile(y1_total, 95, axis=0)
    # y2_95 = np.percentile(y2_total, 95, axis=0)
    # y3_95 = np.percentile(y3_total, 95, axis=0)
    # y4_95 = np.percentile(y4_total, 95, axis=0)

    # print(y1_mean.shape)
    # print(y1_mean)

    # total_robots = y1[0]+y2[0]
    total_robots = TOTAL_ROBOTS

    x = range(0,total_time)
    x = [elem / 10 for elem in x]

    # fig = plt.gcf()
    # fig.set_size_inches(6.5, 3.5)

    font = FontProperties()
    font.set_family('serif')
    font.set_name('Times New Roman')

    plt.figure(figsize=(8,3))

    # plotting the lines
    plt.plot(x, y1_mean, label = "Team 1 Follower")
    plt.plot(x, y2_mean, label = "Team 2 Follower")
    plt.plot(x, y3_mean, label = "Connector")
    if np.amax(y4_mean) > 0:
        plt.plot(x, y4_mean, label = "Traveler")

    # plt.title(title)
    plt.xlabel(x_label, fontsize=15, fontproperties=font)
    plt.ylabel(y_label, fontsize=15, fontproperties=font)

    plt.xticks(np.arange(0, total_time/10, 50), fontsize=14, fontproperties=font)
    # plt.yticks(np.arange(0, total_robots, 5), fontsize=12)
    plt.yticks(np.arange(0, 25, 5), fontsize=14, fontproperties=font)

    plt.xlim([0,360])
    plt.ylim([-0.5,17])

    if np.amax(y4_mean) > 0:
        num_col = 4
    else:
        num_col = 3

    font2 = FontProperties()
    font2.set_family('serif')
    font2.set_name('Times New Roman')
    font2.set_size(14)
    
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc='lower left', ncol=num_col, frameon=False, prop=font2)

    ax = plt.gca()

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(1)
    ax.spines['bottom'].set_linewidth(1)
    ax.tick_params(width=1)

    plt.axhline(y=0, linewidth=1, color='black', linestyle='--')

    # set_size(4.85, 3.4)

    plt.savefig('{}{}.pdf'.format(out_filename.split('.yaml')[0], '_mean'), bbox_inches='tight')

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

    plt.savefig(out_filename, bbox_inches='tight')
    plt.close()


def set_size(w,h, ax=None):
    """ w, h: width, height in inches """
    if not ax: ax=plt.gca()
    l = ax.figure.subplotpars.left
    r = ax.figure.subplotpars.right
    t = ax.figure.subplotpars.top
    b = ax.figure.subplotpars.bottom
    figw = float(w)/(r-l)
    figh = float(h)/(t-b)
    ax.figure.set_size_inches(figw, figh)


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


# def average_completion_time(stats):

#     for scenario, data in stats.items():

#         for timestep in data:

#             demand = 0

#             for task in data[timestep]['tasks']:
#                 demand += task['demand']

#             if demand == 0:
#                 print('Completed at {}'.format(timestep))
#                 return


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


def init_request_time(data):
    for timestep in data:
        for robot in data[timestep]['robots']:
            if robot['state'] == 'LEADER':
                if robot['id'] == 'L1':
                    if robot['action'].split('_')[0] == 'message' and len(robot['action'].split('_')) > 1:
                        return int(timestep.split('_')[1]), int(robot['action'].split('_')[1])


def last_task_time(data):
    for timestep in data:
        demand = 0
        for task in data[timestep]['tasks']:
            demand += task['demand']

        if demand < DEMAND_PER_TASK:
            return int(timestep.split('_')[1])


def distance_since_request(data):
    distance = 0
    request_made = False
    team_2_loc = {}
    for timestep in data:
        # demand = 0
        # for task in data[timestep]['tasks']:
        #     demand += task['demand']

        # if demand == 0:
        #     return distance

        if request_made:
            for robot in data[timestep]['robots']:
                if int(robot['teamid']) == 2 or robot['state'] != 'TRAVELER':
                    if robot['id'] in team_2_loc:
                        pos = robot['pos']
                        prev_pos = team_2_loc[robot['id']]
                        traveled = math.sqrt( ((float(pos['x'])-float(prev_pos['x']))**2)+((float(pos['y'])-float(prev_pos['y']))**2) )
                        # if traveled > 0:
                        #     print('{}, {}'.format(robot['id'], traveled))
                        distance += traveled
                        team_2_loc[robot['id']] = pos
                    else:
                        team_2_loc[robot['id']] = robot['pos']
            # print('{}, Distance: {}'.format(timestep, distance))
        else:
            for robot in data[timestep]['robots']:
                if robot['state'] == 'LEADER':
                    if robot['id'] == 'L1':
                        # print(len(robot['action'].split('_')))
                        if robot['action'].split('_')[0] == 'message' and len(robot['action'].split('_')) > 1:
                            request_made = True

    return distance


def main(argv):
    stats = load_stats(argv)

    num_experiments_success = 0

    all_experiment_time = 0
    all_message_sent = 0
    all_message_received = 0

    all_request_time = 0
    all_robots_requested = 0
    all_distance = 0

    all_started_working_time = 0

    # Plot single stats
    for scenario, data in stats.items():

        print('\n-------- {} --------'.format(scenario))

        # plot_filename = '{0}/robot-states_{1}.pdf'.format(OUTPUT_DIR, scenario)
        # plot_robot_states(scenario,
        #                   data,
        #                   title='Number of robots in each team',
        #                   x_label='Time (seconds)',
        #                   y_label='Number of robots',
        #                   out_filename=plot_filename)
        
        # plot_filename = '{0}/task-demands_{1}.pdf'.format(OUTPUT_DIR, scenario)
        # plot_task_demands(scenario,
        #                   data,
        #                   title='Remaining task demands',
        #                   x_label='Time (seconds)',
        #                   y_label='Task demand',
        #                   out_filename=plot_filename)

        # plot_filename = '{0}/trajectories_{1}.pdf'.format(OUTPUT_DIR, scenario)
        # plot_trajectories(scenario,
        #                   data,
        #                   title='Robot trajectories',
        #                   x_label='X(m)',
        #                   y_label='Y(m)',
        #                   out_filename=plot_filename)

        # Completion time (return int, not complete = None)
        finish_time = len(data)
        print('Finish Time: {}'.format(finish_time))

        # Were tasks completed?
        last_timestep = 'time_{}'.format(len(data))

        is_tasks_completed = True

        for task in data[last_timestep]['tasks']:
            if task['demand'] > 0:
                print('Tasks not completed')
                is_tasks_completed = False
                break

        if is_tasks_completed:
            num_experiments_success += 1
        else:
            continue

        all_experiment_time += finish_time

        # Initial request num and time (return two values)
        # first_request_time, request_num = init_request_time(data)
        # print('First Request Time: {}'.format(first_request_time))
        # print('Robots Requested: {}'.format(request_num))

        # start_last_task_time = last_task_time(data)


        # all_request_time += first_request_time
        # all_robots_requested += request_num

        # Time that the robots started working on the constrained task
        # time_found = False
        # for timestep in data:
        #     for task in data[timestep]['tasks']:
        #         if task['id'] == 'task_3':
        #             if int(task['demand']) < 5000:
        #                 started_working_time = int(timestep.split('_')[1])
        #                 time_found = True
        #                 break
        #     if time_found:
        #         break

        # print('Started working at: {}'.format(started_working_time))
        # all_started_working_time += started_working_time

        # Message ratio
        last_timestep = 'time_{}'.format(len(data))

        for robot in data[last_timestep]['robots']:
            if robot['id'] == 'L1':
                L1_sent = int(robot['total_sent'])
                L1_received = int(robot['total_received'])
            elif robot['id'] == 'L2':
                L2_sent = int(robot['total_sent'])
                L2_received = int(robot['total_received'])

        total_sent = L1_sent + L2_sent
        total_received = L1_received + L2_received
        ratio = total_received / total_sent
        print('Total Sent: {}'.format(total_sent))
        print('Total Received: {}'.format(total_received))
        print('Ratio: {}'.format(ratio))

        all_message_sent += total_sent
        all_message_received += total_received

        # timestep, request_num = init_request(data)
        # print('Init request time: {}'.format(timestep))
        # print('Request Num: {}'.format(request_num))

        # Distance traveled since request first made (distance traveled by teamid != 1)
        # distance = distance_since_request(data)
        # print('Distance: {}'.format(distance))

        # all_distance += distance

    print('\n-------- RESULT SUMMARY --------')

    print('Number of successful experiments: {} of {}'.format(num_experiments_success, len(stats)))

    average_completion_time = all_experiment_time / num_experiments_success   
    print('Average Time: {} -> {}s'.format(average_completion_time, average_completion_time/10))

    average_message_ratio = all_message_received / all_message_sent
    print('Average Ratio: {}'.format(average_message_ratio))

    average_request_time = all_request_time / num_experiments_success
    print('Average Request Time: {} -> {}s'.format(average_request_time, average_request_time/10))

    average_started_working_time = all_started_working_time / num_experiments_success
    print('Average Time Started: {} -> {}s'.format(average_started_working_time, average_started_working_time/10))

    average_waiting_time = average_started_working_time - average_request_time
    print('Average Waiting Time: {} -> {}s'.format(average_waiting_time, average_waiting_time/10))

    average_service_time = average_completion_time - average_request_time
    print('Average Service Time: {} -> {}s'.format(average_service_time, average_service_time/10))

    average_robots_requested = all_robots_requested /  num_experiments_success
    print('Average Ratio: {}'.format(average_robots_requested))

    average_distance = all_distance / num_experiments_success
    print('Average Distance: {}'.format(average_distance))

    # Plot overall stats
    plot_filename = '{0}/overall_robot-states.png'.format(OUTPUT_DIR)
    plot_overall_robot_states(stats,
                              title='Average number of robots in each team',
                              x_label='Time (seconds)',
                              y_label='Number of robots',
                              out_filename=plot_filename)

    # plot_filename = '{0}/overall_task_demands_{1}.pdf'.format(OUTPUT_DIR, next(iter(stats))[:-3])
    # plot_overall_task_demands(stats,
    #                           title='Average remaining task demand',
    #                           x_label='Timestep (seconds)',
    #                           y_label='Task demand',
    #                           out_filename=plot_filename)


    # Run connected ratio separately from the other plotting functions for now as it deals with scenarios with different robot numbers

    # plot_filename = '{0}/overall_connected_ratio.pdf'.format(OUTPUT_DIR)
    # plot_overall_connected_ratio(stats,
    #                              title='Average ratio of beat messages exchanged',
    #                              x_label='Number of follower robots',
    #                              y_label='Ratio of messages received',
    #                              out_filename=plot_filename)


if __name__ == "__main__":
    # plot_stats(sys.argv[1:])
    # argv = [
    #         # '20R_6T_100D_01.yaml', 
    #         # '20R_6T_100D_02.yaml',
    #         # 'template_1.yaml',
    #         # 'template_2.yaml',
    #         # 'template_11.yaml',
    #         # 'template_12.yaml',
    #         'template_13.yaml',
    #        ]
    argv = []
    for i in range(1,51):
    # for i in range(1,26):
    # for i in range(26,51):

        id = ''
        if(i < 10):
            id += '0' + str(i)
        else:
            id += str(i)
        # argv.append('20R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))
        argv.append('30R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))
        # argv.append('40R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

        # argv.append('30R_{0}T_{1}D_no_exchange_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))
        # argv.append('30R_{0}T_{1}D_exchange_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    # for i in range(1,21):
    #     id = ''
    #     if(i < 10):
    #         id += '0' + str(i)
    #     else:
    #         id += str(i)
    #     argv.append('30R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    # for i in range(1,21):
    #     id = ''
    #     if(i < 10):
    #         id += '0' + str(i)
    #     else:
    #         id += str(i)
    #     argv.append('40R_{0}T_{1}D_{2}.yaml'.format(NUMBER_OF_TASKS, DEMAND_PER_TASK, id))

    main(argv)
