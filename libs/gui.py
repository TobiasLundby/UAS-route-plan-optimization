#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: GUI class for the path planner
    License: BSD 3-Clause
"""

from Tkinter import Tk, Label, Button
import threading
from Queue import Queue, Empty
import time

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

    def __init__(self, auto_start = False):
        StoppableThread.__init__(self)
        self.q = Queue()
        if auto_start:
            self.start()

    def callback(self):
        self.root.quit()
        print "Tkinter GUI has stopped but thread will first be joined upon closing of the program"

    def on_main_thread(self, func):
        self.q.put(func)

    def check_queue(self):
        while True:
            try:
                task = self.q.get(block=False)
            except Empty:
                break
            else:
                self.root.after_idle(task)
        self.root.after(100, self.check_queue)

    def set_cnt_label(self, txt):
        com_txt = 'count is %i' % txt
        print 'Tkinter: count is %i' % txt
        self.label_cnt.configure(text=com_txt)

    def greet(self):
        self.label.configure(text="Greetings!")
        self.greet_button.configure(text="Ungreet!")
        self.greet_button.configure(command=self.ungreet)
    def ungreet(self):
        self.label.configure(text="No greetings for you!")
        self.greet_button.configure(text="Greet!")
        self.greet_button.configure(command=self.greet)

    def run(self):
        self.root = Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.label_cnt = Label(self.root, text="Hello World")
        self.label_cnt.pack()

        self.label = Label(self.root, text="")
        self.label.pack()

        self.greet_button = Button(self.root, text="Greet", command=self.greet)
        self.greet_button.pack()

        self.root.after(100, self.check_queue)

        self.root.mainloop()

if __name__ == "__main__":
    pp_gui = path_planner_gui()
    pp_gui.start()

    print('Now we can continue running code while mainloop runs!')

    var = 1
    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)
            var += 1
            print 'Main: count is %i' % var
            pp_gui.on_main_thread( lambda: pp_gui.set_cnt_label(var) )

        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True

    pp_gui.stop()
