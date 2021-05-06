import sys
import os.path
import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse


def plot_stats(argv):

    # Parse csv
    aframe = pd.read_csv(argv[0])

    # Calculate the distance between the two leaders at every timestep
    separation = []
    for index, row in aframe.iterrows():
        dist = pow(pow(row["leader1posX"] - row["leader2posX"], 2) + pow(row["leader1posY"] - row["leader2posY"], 2), 0.5)
        separation.append(dist)
    aframe["separation"] = separation

    # print(aframe)

    # Plot the distance between the leaders
    plot_filename = 'leader-separation.pdf'
    plot_leader_separation(aframe, 
                           title='Separation between leaders',
                           x_label='Timestep',
                           y_label='Distance (m)',
                           out_filename=plot_filename)

    # Plot the number of followers and chains
    plot_filename = 'robot-state.pdf'
    plot_robot_state(aframe,
                     title='Number of robots in each state',
                     x_label='Timestep',
                     y_label='Number of robots',
                     out_filename=plot_filename)


def plot_leader_separation(af, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    plt.figure()

    af.plot(x='# clock', y='separation', legend=None)

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.savefig(out_filename)
    plt.close()


def plot_robot_state(af, title=None, x_label=None, y_label=None, out_filename=None, y_limit=None):
    print('Plotting {0}'.format(out_filename))

    af[['# clock', 'leader1follower', 'leader2follower', 'chain']].plot(x='# clock')

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.savefig(out_filename)
    plt.close()


if __name__ == "__main__":
    plot_stats(sys.argv[1:])
