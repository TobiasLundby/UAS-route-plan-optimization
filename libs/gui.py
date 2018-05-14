#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: GUI class for the path planner
             Note that the Tkinter GUI runs in a different thread and that it
             should be stopped upon program completion
    License: BSD 3-Clause
Inspiration: https://github.com/Akuli/tkinter-tutorial/blob/master/event-loop-stuff.md
             http://stupidpythonideas.blogspot.dk/2013/10/why-your-gui-app-freezes.html
Tkinter examples: https://dzone.com/articles/python-gui-examples-tkinter-tutorial-like-geeks
"""

from Tkinter import Tk, Label, Button, Entry, LEFT
import threading
from Queue import Queue, Empty
import logging

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
    DEFAULT_START_LAT = 55.431122
    DEFAULT_START_LON = 10.420436
    DEFAULT_GOAL_LAT  = 55.427203
    DEFAULT_GOAL_LON  = 10.419043
    def __init__(self, auto_start = False):
        StoppableThread.__init__(self)
        self.q = Queue()

        self.logger = logging.getLogger(__name__)
        if auto_start:
            self.start()

    def stop_thread(self):
        self.callback_close()
        self.stop()

    def callback_close(self):
        self.root.quit()
        self.logger.info('Tkinter GUI has stopped but thread will first be joined upon closing of the program')

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

    def set_label_no_fly_zones(self, txt):
        self.label_data_source_no_fly_zones_res.configure(text=txt)
    def set_label_height_map(self, txt):
        self.label_data_source_height_map_res.configure(text=txt)
    def set_label_drone_id(self, txt):
        self.label_data_source_droneID_res.configure(text=txt)
    def set_global_plan_start_heuristic(self, val):
        self.label_global_plan_start_heuristic_res.configure(text='%.02f' % val)
    def set_global_plan_cur_heuristic(self, val):
        self.label_global_plan_cur_heuristic_res.configure(text='%.02f' % val)
    def set_global_plan_horz_step_size(self, val):
        self.label_global_plan_horz_step_size_res.configure(text='%.01f [m]' % val)
    def set_global_plan_vert_step_size(self, val):
        self.label_global_plan_vert_step_size_res.configure(text='%.01f [m]' % val)
    def set_global_plan_status(self, txt):
        self.label_global_plan_status_res.configure(text=txt)
    def set_global_plan_search_time(self, val):
        self.label_global_plan_search_time_res.configure(text='%.01f' % val)
    def set_global_plan_nodes_visited(self, val):
        self.label_global_plan_nodes_visited_res.configure(text='%i' % val)


    def start_global_path_planning(self):
        print 'Should call some function to start the global path planner'
        self.button_local_plan.configure(state='normal') # this is just for test since it has not called anything yet and the global plan therefore isn't calculated

    def start_local_path_planning(self):
        print 'Should call some function to start the local path planner'

    def run(self):
        self.root = Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback_close)
        self.root.title("UAV Path Planner")
        #self.root.geometry('{}x{}'.format(460, 350))

        # Left side layout
        row_num_left = 0
        self.label_start_point = Label(self.root, text="Path Planning", font=("Arial Bold", 12))
        self.label_start_point.grid(row=row_num_left, column=0, columnspan = 2)

        row_num_left += 1
        self.label_start_point = Label(self.root, text="Start point (geodetic)", font=("Arial Bold", 10))
        self.label_start_point.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_start_point_lat = Label(self.root, text="Latitude [dd]:")
        self.label_start_point_lat.grid(row=row_num_left, column=0)
        self.input_start_point_lat = Entry(self.root,width=10)
        self.input_start_point_lat.insert(0, self.DEFAULT_START_LAT)
        self.input_start_point_lat.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_start_point_lon = Label(self.root, text="Latitude [dd]:")
        self.label_start_point_lon.grid(row=row_num_left, column=0)
        self.input_start_point_lon = Entry(self.root,width=10)
        self.input_start_point_lon.insert(0, self.DEFAULT_START_LON)
        self.input_start_point_lon.grid(row=row_num_left, column=1)

        row_num_left += 1
        self.label_goal_point  = Label(self.root, text="Goal point (geodetic)", font=("Arial Bold", 10))
        self.label_goal_point.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_goal_point_lat = Label(self.root, text="Latitude [dd]:")
        self.label_goal_point_lat.grid(row=row_num_left, column=0)
        self.input_goal_point_lat = Entry(self.root,width=10)
        self.input_goal_point_lat.insert(0, self.DEFAULT_GOAL_LAT)
        self.input_goal_point_lat.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_goal_point_lon = Label(self.root, text="Latitude [dd]:")
        self.label_goal_point_lon.grid(row=row_num_left, column=0)
        self.input_goal_point_lon = Entry(self.root,width=10)
        self.input_goal_point_lon.insert(0, self.DEFAULT_GOAL_LON)
        self.input_goal_point_lon.grid(row=row_num_left, column=1)

        row_num_left += 1
        self.button_global_plan = Button(self.root, text="Start global planning", command=self.start_global_path_planning)
        self.button_global_plan.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_global_plan_status = Label(self.root, text="Status:")
        self.label_global_plan_status.grid(row=row_num_left, column=0)
        self.label_global_plan_status_res = Label(self.root, text="idle")
        self.label_global_plan_status_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_start_heuristic = Label(self.root, text="Start heuristic:")
        self.label_global_plan_start_heuristic.grid(row=row_num_left, column=0)
        self.label_global_plan_start_heuristic_res = Label(self.root, text="N/A")
        self.label_global_plan_start_heuristic_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_cur_heuristic = Label(self.root, text="Best heuristic:")
        self.label_global_plan_cur_heuristic.grid(row=row_num_left, column=0)
        self.label_global_plan_cur_heuristic_res = Label(self.root, text="N/A")
        self.label_global_plan_cur_heuristic_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_horz_step_size = Label(self.root, text="Horizontal step-size:")
        self.label_global_plan_horz_step_size.grid(row=row_num_left, column=0)
        self.label_global_plan_horz_step_size_res = Label(self.root, text="N/A")
        self.label_global_plan_horz_step_size_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_vert_step_size = Label(self.root, text="Vertical step-size:")
        self.label_global_plan_vert_step_size.grid(row=row_num_left, column=0)
        self.label_global_plan_vert_step_size_res = Label(self.root, text="N/A")
        self.label_global_plan_vert_step_size_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_search_time = Label(self.root, text="Search time:")
        self.label_global_plan_search_time.grid(row=row_num_left, column=0)
        self.label_global_plan_search_time_res = Label(self.root, text="N/A")
        self.label_global_plan_search_time_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_global_plan_nodes_visited = Label(self.root, text="Nodes visited:")
        self.label_global_plan_nodes_visited.grid(row=row_num_left, column=0)
        self.label_global_plan_nodes_visited_res = Label(self.root, text="N/A")
        self.label_global_plan_nodes_visited_res.grid(row=row_num_left, column=1)

        row_num_left += 1
        self.button_local_plan = Button(self.root, text="Start local planning", command=self.start_local_path_planning)
        self.button_local_plan.configure(state='disabled') # disable the button since it cannot make a local plan before it has made a global plan
        self.button_local_plan.grid(row=row_num_left, column=0, columnspan = 2)

        # Right side layout
        row_num_right = 0
        self.label_data_sources = Label(self.root, text="Data data sources", font=("Arial Bold", 10))
        self.label_data_sources.grid(row=row_num_right, column=3, columnspan = 2)

        row_num_right += 1
        self.label_data_source_no_fly_zones = Label(self.root, text="No-fly zones:")
        self.label_data_source_no_fly_zones.grid(row=row_num_right, column=3)
        self.label_data_source_no_fly_zones_res = Label(self.root, text="not loaded")
        self.label_data_source_no_fly_zones_res.grid(row=row_num_right, column=4)

        row_num_right += 1
        self.label_data_source_height_map = Label(self.root, text="Height map:")
        self.label_data_source_height_map.grid(row=row_num_right, column=3)
        self.label_data_source_height_map_res = Label(self.root, text="not loaded")
        self.label_data_source_height_map_res.grid(row=row_num_right, column=4)

        row_num_right += 1
        self.label_data_source_droneID = Label(self.root, text="DroneID:")
        self.label_data_source_droneID.grid(row=row_num_right, column=3)
        self.label_data_source_droneID_res = Label(self.root, text="not loaded")
        self.label_data_source_droneID_res.grid(row=row_num_right, column=4)


        # Configure the queue callback
        self.root.after(100, self.check_queue)

        # Start the main loop
        self.root.mainloop()

if __name__ == "__main__":
    pp_gui = path_planner_gui()
    pp_gui.start()

    import time

    var = 1
    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)

        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True
    pp_gui.on_main_thread( lambda: pp_gui.set_label_no_fly_zones('loaded') )
    pp_gui.on_main_thread( lambda: pp_gui.set_label_height_map('loaded') )

    pp_gui.stop_thread()
