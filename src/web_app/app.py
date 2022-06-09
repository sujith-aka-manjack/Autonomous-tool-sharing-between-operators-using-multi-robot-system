# Based on 
# - https://www.yukiyukiponsu.work/entry/python-Flask-btn-page-move
# - https://stackoverflow.com/a/49334973

from flask import Flask, render_template, request
import socket
from worker import SimulationProcess, WebClientProcess

import os
import os.path
import argparse

# Scenarios
SCENARIO_TRAINING = "experiments/webviz_training.argos"
SCENARIO_TRIAL1 = "experiments/user_study_scenario1.argos"
SCENARIO_TRIAL2 = "experiments/user_study_scenario2.argos"

SCENARIO_TESTMULTIOP = "experiments/webviz_multi-op.argos"
SCENARIO_TESTTRIAL = "experiments/test/test_user_study.argos"

# Local machine IP address
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ip_addr = s.getsockname()[0]
simulation_link = "{}:8000".format(ip_addr)

# Store IP address
# if(not os.path.isdir('results')):
#     os.mkdir('results')
# f = open('results/ip_address.txt', 'w')
# f.write(ip_addr)
# f.close()

# Currently running simulation scenario
proc_simulation = None
proc_webclient  = None

mode = None
# order = None

app = Flask(__name__)


# Access to "/": redirect to start_page.html
@app.route("/")
def hello():
    return render_template("start_page.html")


# Access to "/startpage": redirect to start_page.html
@app.route("/startpage", methods=["GET"])
def startpage():
    return render_template("start_page.html")


# Access to "/trainingpage": redirect to training_page.html
@app.route("/trainingpage", methods=["GET"])
def trainingpage():
    return render_template("training_page.html", mode=mode)


# Access to "/trial1page": redirect to trial1_page.html
@app.route("/trial1page", methods=["GET"])
def trial1page():
    return render_template("trial1_page.html", mode=mode)


# Access to "/trial2page": redirect to trial2_page.html
@app.route("/trial2page", methods=["GET"])
def trial2page():
    return render_template("trial2_page.html", mode=mode)


# # Access to "/trainingpage": redirect to training_page.html
# @app.route("/testmultioppage", methods=["GET"])
# def testmultioppage():
#     return render_template("test-multi-op_page.html")


# Access to "/endpage": redirect to end_page.html
@app.route("/endpage", methods=["GET"])
def endpage():
    return render_template("end_page.html")


# Background process: Start the simulation
@app.route('/background_process_start', methods=['POST'])
def background_process_start():
    scenario = request.get_data().decode('UTF-8')
    print ("Start '{}'".format(scenario))

    global proc_simulation, proc_webclient

    if(scenario == 'training'):
        proc_simulation = SimulationProcess(SCENARIO_TRAINING)        
    elif(scenario == 'trial1'):
        proc_simulation = SimulationProcess(SCENARIO_TRIAL1)
    elif(scenario == 'trial2'):
        proc_simulation = SimulationProcess(SCENARIO_TRIAL2)
    elif(scenario == 'test-multi-op'):
        proc_simulation = SimulationProcess(SCENARIO_TESTMULTIOP)
    else:
        print('No scenario provided.')

    proc_simulation.start()
    proc_webclient = WebClientProcess()
    proc_webclient.start()
    return ("nothing")


# Background process: Stop the simulation
@app.route('/background_process_stop')
def background_process_stop():
    print ("Stop")
    global proc_simulation, proc_webclient
    if proc_simulation:
        proc_simulation.stop()
    if proc_webclient:
        proc_webclient.stop()
    return ("nothing")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate experiment files for ARGoS simulations.')

    parser.add_argument('-m', '--mode', type=str,
                        choices=['indirect', 'direct', 'debug'],
                        help='The communication mode to use in the simulation (choose "indirect" for default capability).',
                        default='indirect')

    # parser.add_argument('-o', '--order', type=int,
    #                     choices=range(1, 3),
    #                     help='The order in which the tasks will be presented.',
    #                     default=1)
    
    args = parser.parse_args()
    if args.mode == 'indirect':
        mode = 'ind'
    elif args.mode == 'direct':
        mode = 'dir'
    else:
        mode = args.mode
        
    # order = args.order

    print('------------------')
    print('Mode\t: {}'.format(mode))
    # print('Order\t: {}'.format(order))
    print('------------------')

    app.run(debug=False, host='0.0.0.0')