from os import listdir, environ
from os.path import isdir, join, splitext
import numpy as np
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import time
from collections import defaultdict

from load_pb import SimData

import sys

OUTPUT_DIR = join(environ['HOME'], 'GIT/argos-sct/results')
BINARY_FILENAME = 'log_data.pb'

def load_stats(dirPath):
    print(dirPath)
    all_dirs = [f for f in listdir(dirPath) if isdir(join(dirPath, f))]
    all_dirs.sort()

    print('Parsing experiments ...')

    start_time = time.time()
    stats = {}

    for trial_dir in all_dirs:
        start_time_single = time.time()

        file = join(trial_dir, 'log_data.pb')

        scenario = splitext(file)[0]
        s = SimData(join(dirPath, file))
        stats[scenario] = s

        duration_single = round(time.time() - start_time_single, 3)
        duration_total = round(time.time() - start_time, 3)
        print('Loaded {0} in {1} s ({2} s)'.format(scenario, duration_single, duration_total))
    
    duration = round(time.time() - start_time, 3)
    print('Finished loading in {0} seconds'.format(duration))

    return stats


def plot_overall_robot_states(stats, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    y1_total = []
    y2_total = []
    y3_total = []
    y4_total = []

    longestTime = 0
    for scenario, sim_data in stats.items():
        if sim_data.totalTime > longestTime:
            longestTime = sim_data.totalTime

    for scenario, sim_data in stats.items():
        y1 = []
        y2 = []
        y3 = []
        y4 = []

        for time, step in sim_data.data.items():
            # Counters
            num_followers_team1 = 0
            num_followers_team2 = 0
            num_connectors = 0
            num_travelers = 0

            for robot in step.robots:
                if robot.state == 0: # FOLLOWER
                    if robot.teamID == 1:
                        num_followers_team1 += 1
                    elif robot.teamID == 2:
                        num_followers_team2 += 1
                elif robot.state == 2: # CONNECTOR
                    num_connectors += 1
                elif robot.state == 3: # TRAVELER
                    num_travelers += 1

            # Fill data for timestep 0
            # if time == 1:
            #     y1.append(num_followers_team1)
            #     y2.append(num_followers_team2)
            #     y3.append(num_connectors)
            #     y4.append(num_travelers)

            y1.append(num_followers_team1)
            y2.append(num_followers_team2)
            y3.append(num_connectors)
            y4.append(num_travelers)

            # print(y1)
            # print(y2)
            # print(y3)
            # print(y4)

            # sys.exit(0)

        y1_total.append(y1)
        y2_total.append(y2)
        y3_total.append(y3)
        y4_total.append(y4)

        # Fill empty timesteps with last value
        for states in [y1_total, y2_total, y3_total, y4_total]:
            for y in states:
                if(len(y) < longestTime):
                    last_val = y[-1]
                    for i in range(0,longestTime - len(y)):
                        y.append(last_val)  

    y1_total = np.array(y1_total)
    y2_total = np.array(y2_total)
    y3_total = np.array(y3_total)
    y4_total = np.array(y4_total)

    print(y1_total.shape)
    print(len(y1_total[0]))
    print(len(y1_total[1]))
    print(len(y1_total[2]))

    # sys.exit(0)

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

    x = range(0, longestTime)
    x = [elem / 10 for elem in x]

    # print(y1_mean.shape)
    # print(y1_mean)
    # sys.exit(0)

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

    plt.xticks(np.arange(0, sim_data.totalTime/10, 50), fontsize=14, fontproperties=font)
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

    all_final_connectors = 0

    for scenario, data in stats.items():
        print('\n-------- {} --------'.format(scenario))

        # Completion time
        finish_time = data.totalTime
        print('finish time: {}'.format(finish_time))

        # Task success
        is_tasks_completed = True
        for task in data[finish_time].tasks:
            if task.demand > 0:
                is_tasks_completed = False
                break

        if is_tasks_completed:
            num_experiments_success += 1
        print('success {}'.format(is_tasks_completed))

        all_experiment_time += finish_time

    print('\n-------- RESULT SUMMARY --------')

    print('Number of successful experiments: {} of {}'.format(num_experiments_success, len(stats)))

    if(num_experiments_success > 0):
        average_completion_time = all_experiment_time / num_experiments_success   
        print('Average Time: {} -> {}s'.format(average_completion_time, average_completion_time/10))
    else:
        print('Average Time: No trials succeeded')

    # Plot overall stats
    scenario_name = dirPath.split('/')[-1]
    plot_filename = '{0}/{1}/overall_robot-states.pdf'.format(OUTPUT_DIR, scenario_name)
    plot_overall_robot_states(stats,
                              title='Average number of robots in each team',
                              x_label='Time (seconds)',
                              y_label='Number of robots',
                              out_filename=plot_filename)


if __name__ == "__main__":
    
    dirPath = "/home/genki/GIT/argos-sct/results/user_study_scenario1_order2"
    main(dirPath)
