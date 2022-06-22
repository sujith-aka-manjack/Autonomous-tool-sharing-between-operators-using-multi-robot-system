# Based on 
# - https://www.yukiyukiponsu.work/entry/python-Flask-btn-page-move
# - https://stackoverflow.com/a/49334973

from flask import Flask, render_template, request, session, redirect, url_for
import socket
from websocket import create_connection
from worker import SimulationProcess, WebClientProcess

import os
import os.path
import argparse
import logging

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

# Simulation mode
mode = None

app = Flask(__name__)

# Based on https://testdriven.io/blog/flask-sessions/
#
# Details on the Secret Key: https://flask.palletsprojects.com/en/2.0.x/config/#SECRET_KEY
# NOTE: The secret key is used to cryptographically-sign the cookies used for storing
#       the session data.
# Command to generate a key:
#     $ python -c 'import secrets; print(secrets.token_hex())'
app.secret_key = '455f3bcd02702ffca86e711f6e176b5983326372b6f04fa5ea66a97bdb3b9e95'

# Disable web access logging in terminal
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
# app.logger.disabled = True
# log.disabled = True


# Access to "/": redirect to start_page.html
@app.route("/")
def default():
    return redirect(url_for('startpage'))


# Access to "/startpage": redirect to start_page.html
@app.route("/startpage", methods=["GET"])
def startpage():
    if 'username' in session:
        return render_template("start_page.html", unique_id=session['username'])
    else:
        return render_template("start_page.html")


# Access to "/trainingpage": redirect to training_page.html
@app.route("/trainingpage", methods=["GET"])
def trainingpage():
    if 'username' in session:
        return render_template("training_page.html", mode=mode, unique_id=session['username'])
    else:
        return redirect(url_for('startpage'))


# Access to "/trial1page": redirect to trial1_page.html
@app.route("/trial1page", methods=["GET"])
def trial1page():
    if 'username' in session:
        return render_template("trial1_page.html", mode=mode, unique_id=session['username'])
    else:
        return redirect(url_for('startpage'))


# Access to "/trial2page": redirect to trial2_page.html
@app.route("/trial2page", methods=["GET"])
def trial2page():
    if 'username' in session:
        return render_template("trial2_page.html", mode=mode, unique_id=session['username'])
    else:
        return redirect(url_for('startpage'))


# # Access to "/trainingpage": redirect to training_page.html
# @app.route("/testmultioppage", methods=["GET"])
# def testmultioppage():
#     return render_template("test-multi-op_page.html")


# Access to "/endpage": redirect to end_page.html
@app.route("/endpage", methods=["GET"])
def endpage():
    if 'username' in session:
        return render_template("end_page.html")
    else:
        return redirect(url_for('startpage'))


# Clear the username stored in the session object
@app.route('/delete')
def delete_username():
    session.pop('username', default=None)
    return redirect(url_for('startpage'))


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


# Background process: Record the user's ID
@app.route('/background_process_record_id', methods=['POST'])
def background_process_record_id():
    session['username'] = request.get_data().decode('UTF-8')
    print ("Received id: {}".format(session['username']))
    return ("nothing")


# Check if the local simulation is running
@app.route('/connection_status', methods=['GET'])
def connect_to_server():
    try:
        ws = create_connection("ws://0.0.0.0:3000")
        ws.recv()
        ws.close()
        return "running"
    except ConnectionRefusedError as error:
        # print("Could not connect to local simulation")
        return "terminated"


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