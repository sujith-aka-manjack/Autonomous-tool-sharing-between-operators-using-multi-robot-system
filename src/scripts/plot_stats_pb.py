from os import listdir
from os.path import isfile, join, splitext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import time
from collections import defaultdict

from load_pb import SimData


def main(dirPath):

    files = [f for f in listdir(dirPath) if isfile(join(dirPath, f))]

    print('Parsing experiments ...')

    start_time = time.time()
    stats = {}

    for file in files:
        start_time_single = time.time()

        scenario = splitext(file)[0]
        s = SimData(join(dirPath, file))
        stats[scenario] = s

        duration_single = round(time.time() - start_time_single, 3)
        duration_total = round(time.time() - start_time, 3)
        print('Loaded {0} in {1} s ({2} s)'.format(scenario, duration_single, duration_total))
    
    duration = round(time.time() - start_time, 3)
    print('Finished loading in {0} seconds'.format(duration))


if __name__ == "__main__":
    
    dirPath = "/home/genki/GIT/argos-sct/results/test_pb/"
    main(dirPath)
