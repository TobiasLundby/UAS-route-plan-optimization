#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: GUI class for the path planner
    License: BSD 3-Clause
"""

import threading
from Tkinter import Tk, Label, Button

root = Tk() # create master/root object

class StoppableThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()

    def stop(self):
        if self.isAlive() == True:
            # set event to signal thread to terminate
            self.stop_event.set()
            # block calling thread until thread really has terminated
            self.join()

class path_planner_gui(StoppableThread):
    def __init__(self, master):
        StoppableThread.__init__(self)
        self.master = master
        master.title("UAV Path Planning")

        self.label = Label(master, text="This is our first GUI!")
        self.label.pack()

        self.greet_button = Button(master, text="Greet", command=self.greet)
        self.greet_button.pack()

        self.close_button = Button(master, text="Close", command=master.quit)
        self.close_button.pack()

    def run(self):
        self.master.mainloop()

    def greet(self):
        self.label.configure(text="It works")
        print("Greetings!")

if __name__ == "__main__":
    pp_gui = path_planner_gui(root)
    pp_gui.start()

    #root.update()
    print 1+1

    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)
            print 1+1
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True

    pp_gui.stop()
