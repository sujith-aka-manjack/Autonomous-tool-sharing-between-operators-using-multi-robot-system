import os.path
import argparse
import datetime
import random
from xml.dom import minidom
import xml.etree.ElementTree as ET

TEMPLATE_PATH = os.path.join(os.environ['HOME'], 'GIT/argos-sct/experiments/experiment_template.argos')


def generate_experiments(num_robots, duration, argos_dir_path, log_dir_path):
    # Load template experiment file
    xmlTree = ET.parse(TEMPLATE_PATH)
    rootElement = xmlTree.getroot()

    # Change time and seed attribute values
    for framework_elem in rootElement.findall("framework"):
        for experiment_elem in framework_elem.findall("experiment"):
            print(experiment_elem.attrib['length'])
            
            experiment_elem.set('length', duration)
            seed = str(random.randint(0,100000))
            print(seed)
            experiment_elem.set('random_seed', seed)
            break

    # Change the output path
    for loop_func_elem in rootElement.findall('loop_functions'):
        for output_elem in loop_func_elem.findall('output'):
            output_elem.set('out_path', log_dir_path)

    # Change the number of worker robots
    num_robots_per_team = int(int(num_robots) / 2)
    for loop_func_elem in rootElement.findall('loop_functions'):
        for teams_elem in loop_func_elem.findall('teams'):
            for team_elem in teams_elem.findall('team'):
                team_elem.set('robot_num', str(num_robots_per_team))

    xmlTree.write(os.path.join(argos_dir_path, 'newitems.xml'))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate experiment files for ARGoS simulations.')

    parser.add_argument('robots',
                        help='Number of worker robots.')

    parser.add_argument('duration',
                        help='Experiment duration in seconds (1 second = 10 timesteps).')

    parser.add_argument('argos_dir_path',
                        help='Directory path to store the argos files')

    parser.add_argument('log_dir_path',
                        help='Directory path to store the log files')

    args = parser.parse_args()

    num_robots = args.robots
    duration = args.duration
    argos_dir_path = args.argos_dir_path
    log_dir_path = args.log_dir_path

    generate_experiments(num_robots, duration, argos_dir_path, log_dir_path)