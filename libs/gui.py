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

from Tkinter import Tk, Label, Button, Entry, LEFT, INSERT, END
from ttk import Combobox
import ScrolledText as tkst
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
    DEFAULT_START_LAT = 55.435259 #55.431122
    DEFAULT_START_LON = 10.410862 #10.420436
    DEFAULT_GOAL_LAT  = 55.424736 #55.427203
    DEFAULT_GOAL_LON  = 10.419749 #10.419043
    def __init__(self, parent_class, auto_start = False):
        # self.__class__ = type(self.__class__.__name__, (base_class, object), dict(self.__class__.__dict__))
        # super(self.__class__, self).__init__()
        self.parent_class = parent_class

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

    def set_label_no_fly_zones(self, txt, color = 'black'):
        self.label_data_source_no_fly_zones_res.configure(text=txt)
        self.label_data_source_no_fly_zones_res.configure(fg=color)
    def set_label_height_map(self, txt, color = 'black'):
        self.label_data_source_height_map_res.configure(text=txt)
        self.label_data_source_height_map_res.configure(fg=color)
    def set_label_drone_id(self, txt, color = 'black'):
        self.label_data_source_droneID_res.configure(text=txt)
        self.label_data_source_droneID_res.configure(fg=color)
    def set_label_adsb(self, txt, color = 'black'):
        self.label_data_source_adsb_res.configure(text=txt)
        self.label_data_source_adsb_res.configure(fg=color)
    def set_label_weather(self, txt, color = 'black'):
        self.label_data_source_weather_res.configure(text=txt)
        self.label_data_source_weather_res.configure(fg=color)
    def set_global_plan_start_heuristic(self, val, color = 'black'):
        self.label_global_plan_start_heuristic_res.configure(text='%.02f' % val)
        self.label_global_plan_start_heuristic_res.configure(fg=color)
    def set_global_plan_cur_heuristic(self, val, color = 'black'):
        self.label_global_plan_cur_heuristic_res.configure(text='%.02f' % val)
        self.label_global_plan_cur_heuristic_res.configure(fg=color)
    def set_global_plan_horz_step_size(self, val, color = 'black'):
        self.label_global_plan_horz_step_size_res.configure(text='%.01f [m]' % val)
        self.label_global_plan_horz_step_size_res.configure(fg=color)
    def set_global_plan_vert_step_size(self, val, color = 'black'):
        self.label_global_plan_vert_step_size_res.configure(text='%.01f [m]' % val)
        self.label_global_plan_vert_step_size_res.configure(fg=color)
    def set_global_plan_status(self, txt, color = 'black'):
        self.label_global_plan_status_res.configure(text=txt)
        self.label_global_plan_status_res.configure(fg=color)
    def set_global_plan_search_time(self, val, color = 'black'):
        self.label_global_plan_search_time_res.configure(text='%.01f [s]' % val)
        self.label_global_plan_search_time_res.configure(fg=color)
    def set_global_plan_nodes_visited(self, val, color = 'black'):
        self.label_global_plan_nodes_visited_res.configure(text='%i' % val)
        self.label_global_plan_nodes_visited_res.configure(fg=color)
    def set_global_plan_nodes_explored(self, val, color = 'black'):
        self.label_global_plan_nodes_explored_res.configure(text='%i' % val)
        self.label_global_plan_nodes_explored_res.configure(fg=color)

    def set_scrolledtext_global_path(self, txt):
        self.scrolledtext_global_path.delete(1.0,END)
        self.scrolledtext_global_path.insert(INSERT,txt)

    def global_planner_thread(self, point_start, point_goal, path_planner):
        if self.parent_class.plan_path_global(point_start, point_goal, path_planner): # Plan path and test the result to update the GUI
            self.button_local_plan.configure(state='normal')
            self.button_global_plan.configure(state='normal')
            self.button_global_plan.configure(text='Start global planning')
            self.button_show_result_webpage.configure(state='normal')
        else: # The global path planner failed and therefore diable the local path planner and change the option to continue
            self.button_local_plan.configure(state='disabled')
            self.button_global_plan.configure(state='normal')
            self.button_show_result_webpage.configure(state='disabled')
            self.button_global_plan.configure(text='Continue global planning')

    def start_global_path_planning(self):
        self.button_global_plan.configure(state='disabled')
        self.button_local_plan.configure(state='disabled')
        self.button_show_result_webpage.configure(state='disabled')

        # Get data from the GUI
        start_point_3dDICT = {'lat': float(self.input_start_point_lat.get()), 'lon': float(self.input_start_point_lon.get()), 'alt_rel': 0}
        goal_point_3dDICT  = {'lat': float(self.input_goal_point_lat.get()), 'lon': float(self.input_goal_point_lon.get()), 'alt_rel': 0}
        path_planner = self.combo_planner_type.get()

        # Create and start the thread
        thread_global_planning = threading.Thread(target=self.global_planner_thread, args=(start_point_3dDICT, goal_point_3dDICT, path_planner))
        thread_global_planning.start()
        #thread_global_planning.join()

    def start_local_path_planning(self):
        print 'Should call some function to start the local path planner'

    def show_result_webpage(self):
        self.parent_class.map_plotter.show_plot()

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
        self.label_planner_type = Label(self.root, text="Type:")
        self.label_planner_type.grid(row=row_num_left, column=0)
        self.combo_planner_type = Combobox(self.root)
        self.combo_planner_type['values'] = self.parent_class.PATH_PLANNER_NAMES
        self.combo_planner_type.current(0)
        self.combo_planner_type.grid(row=row_num_left, column=1)

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
        self.label_global_plan_nodes_explored = Label(self.root, text="Nodes explored:")
        self.label_global_plan_nodes_explored.grid(row=row_num_left, column=0)
        self.label_global_plan_nodes_explored_res = Label(self.root, text="N/A")
        self.label_global_plan_nodes_explored_res.grid(row=row_num_left, column=1)


        row_num_left += 1
        self.button_local_plan = Button(self.root, text="Start local planning", command=self.start_local_path_planning)
        self.button_local_plan.configure(state='disabled') # disable the button since it cannot make a local plan before it has made a global plan
        self.button_local_plan.grid(row=row_num_left, column=0, columnspan = 2)

        row_num_left += 1
        self.button_show_result_webpage = Button(self.root, text="Show result webpage", command=self.show_result_webpage)
        self.button_show_result_webpage.configure(state='disabled') # Disabled because no global plan has been made
        self.button_show_result_webpage.grid(row=row_num_left, column=0, columnspan = 2)

        # Right side layout
        row_num_right = 0
        self.label_data_sources = Label(self.root, text="Data data sources", font=("Arial Bold", 10))
        self.label_data_sources.grid(row=row_num_right, column=2, columnspan = 2)

        row_num_right += 1
        self.label_data_source_no_fly_zones = Label(self.root, text="No-fly zones:")
        self.label_data_source_no_fly_zones.grid(row=row_num_right, column=2)
        self.label_data_source_no_fly_zones_res = Label(self.root, text="not loaded")
        self.label_data_source_no_fly_zones_res.configure(fg='red')
        self.label_data_source_no_fly_zones_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        self.label_data_source_height_map = Label(self.root, text="Height map:")
        self.label_data_source_height_map.grid(row=row_num_right, column=2)
        self.label_data_source_height_map_res = Label(self.root, text="not loaded")
        self.label_data_source_height_map_res.configure(fg='red')
        self.label_data_source_height_map_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        self.label_data_source_droneID = Label(self.root, text="DroneID:")
        self.label_data_source_droneID.grid(row=row_num_right, column=2)
        self.label_data_source_droneID_res = Label(self.root, text="not loaded")
        self.label_data_source_droneID_res.configure(fg='red')
        self.label_data_source_droneID_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        self.label_data_source_adsb = Label(self.root, text="ADS-B:")
        self.label_data_source_adsb.grid(row=row_num_right, column=2)
        self.label_data_source_adsb_res = Label(self.root, text="not loaded")
        self.label_data_source_adsb_res.configure(fg='red')
        self.label_data_source_adsb_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        self.label_data_source_weather = Label(self.root, text="Weather:")
        self.label_data_source_weather.grid(row=row_num_right, column=2)
        self.label_data_source_weather_res = Label(self.root, text="not loaded")
        self.label_data_source_weather_res.configure(fg='red')
        self.label_data_source_weather_res.grid(row=row_num_right, column=3)

        # Both sides
        row_num_left += 1
        self.label_global_path = Label(self.root, text="Global path:")
        self.label_global_path.grid(row=row_num_left, column=0)
        self.scrolledtext_global_path = tkst.ScrolledText(self.root,height=10)
        self.scrolledtext_global_path.grid(row=row_num_left, column=1, columnspan=3)


        # Configure the queue callback
        self.root.after(250, self.check_queue)

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
