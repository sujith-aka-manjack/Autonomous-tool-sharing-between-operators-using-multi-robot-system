# Based on https://stackoverflow.com/questions/7546050/switch-between-two-frames-in-tkinter

try:
    import tkinter as tk                # python 3
    from tkinter import font as tkfont  # python 3
except ImportError:
    import Tkinter as tk     # python 2
    import tkFont as tkfont  # python 2

import socket
import pyperclip as pc

from worker import SimulationProcess, WebClientProcess

# Path to ARGoS binary
ARGOS = '/usr/bin/argos3'

# Scenarios
SCENARIO_TRAINING = "experiments/webviz_training.argos"

class ExperimentApp(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.title_font = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")
        self.body_font = tkfont.Font(family='Helvetica', size=12, weight="normal", slant="roman")

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Get ip address of local machine
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_addr = s.getsockname()[0]
        self.simulation_link = "{}:8000".format(ip_addr)

        # Currently running simulation scenario
        self.proc_simulation = None
        self.proc_webclient  = None

        # Create each page
        page_names = [StartPage, PageOne, EndPage]
        self.frames = {}

        for F in page_names:
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("StartPage")

        # register handler for when the window is closed
        self.protocol("WM_DELETE_WINDOW", self.on_window_deleted)

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()

    def start(self, scenario):
        print('start pressed')
        self.stop()

        self.proc_simulation = SimulationProcess(scenario)
        self.proc_simulation.start()
        self.proc_webclient = WebClientProcess()
        self.proc_webclient.start()

    def stop(self):
        print('stop called')
        if self.proc_simulation:
            self.proc_simulation.stop()

        if self.proc_webclient:
            self.proc_webclient.stop()

    def quit(self):
        print('quit pressed')
        self.stop()
        self.destroy()

    def on_window_deleted(self):
        print('window closed')
        self.stop()
        self.destroy()     


class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        label_title = tk.Label(self, text="Welcome!", font=controller.title_font)
        label_title.pack(side="top", fill="x", pady=10)

        label_description = tk.Label(self, text="This is the description.", font=controller.body_font)
        label_description.pack(padx=10)

        # Bottom Frame
        bottom_frame = tk.Frame(self)
        bottom_frame.pack(side="bottom", pady=10)

        button_next = tk.Button(bottom_frame, text="Next",
                                command=lambda: controller.show_frame("PageOne"))
        button_next.pack(side="right")


class PageOne(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        label_title = tk.Label(self, text="Experiment 1", font=controller.title_font)
        label_title.pack(side="top", fill="x", pady=10)

        # Center Frame
        center_frame = tk.Frame(self)
        center_frame.pack(pady=10)

        label_step1 = tk.Label(center_frame, text="1. Press the Start button to begin the experiment.", font=controller.body_font)
        label_step1.pack(padx=10)

        button_start = tk.Button(center_frame, text="Start",
                                 command=lambda: controller.start(SCENARIO_TRAINING))
        button_start.pack()

        label_step2 = tk.Label(center_frame, text="2. Open a browser and access: {}".format(controller.simulation_link), font=controller.body_font)
        label_step2.pack(padx=10, pady=(10,0))

        button_copy_link = tk.Button(center_frame, text="Copy Link",
                                     command=lambda: pc.copy(controller.simulation_link))
        button_copy_link.pack()

        label_step3 = tk.Label(center_frame, text="3. Press the Stop button when finished.", font=controller.body_font)
        label_step3.pack(padx=10, pady=(10,0))

        button_stop = tk.Button(center_frame, text="Stop",
                                command=lambda: controller.stop())
        button_stop.pack()

        # Bottom Frame
        bottom_frame = tk.Frame(self)
        bottom_frame.pack(side="bottom", pady=10)

        button_next = tk.Button(bottom_frame, text="Next",
                                command=lambda: controller.show_frame("EndPage"))
        button_back = tk.Button(bottom_frame, text="Back",
                                command=lambda: controller.show_frame("StartPage"))
        button_next.pack(side="right")
        button_back.pack(side="left")


class EndPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        label_title = tk.Label(self, text="Thank you!", font=controller.title_font)
        label_title.pack(side="top", fill="x", pady=10)

        label_description = tk.Label(self, text="This is the description.", font=controller.body_font)
        label_description.pack(padx=10)

        # Bottom Frame
        bottom_frame = tk.Frame(self)
        bottom_frame.pack(side="bottom", pady=10)

        button_quit = tk.Button(bottom_frame, text="Quit",
                           command=lambda: controller.quit())
        button_back = tk.Button(bottom_frame, text="Back",
                           command=lambda: controller.show_frame("PageOne"))
        button_quit.pack(side="right")
        button_back.pack(side="left")


if __name__ == "__main__":
    app = ExperimentApp()
    app.mainloop()