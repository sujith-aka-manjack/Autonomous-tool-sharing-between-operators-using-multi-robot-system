# Based on https://github.com/Terrabits/tkinter-multiprocessing-example/blob/master/lib/worker.py

import ctypes
import multiprocessing
import time
import subprocess
import signal

ARGOS = '/usr/bin/argos3'

class Worker(object):
    def __init__(self, scenario):
        self._is_alive  = multiprocessing.Value(ctypes.c_bool, True)
        self.scenario   = scenario
        self.simulation = None

    def get_is_alive(self):
        with self._is_alive.get_lock():
            return self._is_alive.value
    def set_is_alive(self, value):
        with self._is_alive.get_lock():
            self._is_alive.value = value
    is_alive = property(get_is_alive, set_is_alive)

    def run(self):
        self.simulation = subprocess.Popen([ARGOS, '-c', self.scenario])
        print("{}".format(self.scenario))

        while True:
            if not self.is_alive:
                self.simulation.send_signal(signal.SIGINT)
                return
            time.sleep(1)

class Process(object):
    def __init__(self, scenario):
        self.worker   = None
        self.process  = None
        self.scenario = scenario
    def __delete__(self):
        self.stop()

    def start(self):
        self.stop()
        self.worker  = Worker(self.scenario)
        self.process = multiprocessing.Process(target=self.worker.run)
        self.process.start()

    def stop(self):
        if self.is_alive:
            self.worker.is_alive = False
            self.process.join()
            self.worker  = None
            self.process = None
    def get_is_alive(self):
        return bool(self.worker and self.process)
    is_alive = property(get_is_alive)