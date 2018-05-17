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
import webbrowser

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
    DEFAULT_START_LAT = 55.43526 #55.431122
    DEFAULT_START_LON = 10.41086 #10.420436
    DEFAULT_GOAL_LAT  = 55.42474 #55.427203
    DEFAULT_GOAL_LON  = 10.41975 #10.419043

    DEFAULT_STEP_SIZE_HORZ = 75
    DEFAULT_STEP_SIZE_VERT = 10
    DEFAULT_SEARCH_TIME_MAX = 120 # unit: s

    DEFAULT_TIME_STEP = 1.0 # unit: s
    DEFAULT_ACCELERATION_FACTOR = 4.0 # unitless
    DEFAULT_STEP_SIZE_HORZ_LOCAL = 25
    DEFAULT_STEP_SIZE_VERT_LOCAL = 10
    DEFAULT_SEARCH_TIME_MAX_LOCAL = 180 # unit: s

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
    def set_label_rally_points(self, txt, color = 'black'):
        self.label_data_rally_point_res.configure(text=txt)
        self.label_data_rally_point_res.configure(fg=color)
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
    def set_label_gpe_fitness(self, val, color = 'black'):
        self.label_gpe_fitness_res.configure(text='%f'%val)
        self.label_gpe_fitness_res.configure(fg=color)
    def set_label_gpe_dist_tot(self, val, color = 'black'):
        self.label_gpe_dist_tot_res.configure(text='%.02f [m]'%val)
        self.label_gpe_dist_tot_res.configure(fg=color)
    def set_label_gpe_dist_horz(self, val, color = 'black'):
        self.label_gpe_dist_horz_res.configure(text='%.02f [m]'%val)
        self.label_gpe_dist_horz_res.configure(fg=color)
    def set_label_gpe_dist_vert(self, val, color = 'black'):
        self.label_gpe_dist_vert_res.configure(text='%.02f [m]'%val)
        self.label_gpe_dist_vert_res.configure(fg=color)
    def set_label_gpe_eta(self, val, color = 'black'):
        self.label_gpe_eta_res.configure(text='%.02f [s]'%val)
        self.label_gpe_eta_res.configure(fg=color)
    def set_label_gpe_wps(self, val, color = 'black'):
        self.label_gpe_wps_res.configure(text='%i'%val)
        self.label_gpe_wps_res.configure(fg=color)
    def set_label_gpe_runtime(self, val, color = 'black'):
        self.label_gpe_runtime_res.configure(text='%.02f [s]'%val)
        self.label_gpe_runtime_res.configure(fg=color)
    def set_label_gpe_bytes_tot(self, val, color = 'black'):
        self.label_gpe_bytes_tot_res.configure(text='%i'%val)
        self.label_gpe_bytes_tot_res.configure(fg=color)
    def set_label_gpe_objects_tot(self, val, color = 'black'):
        self.label_gpe_objects_tot_res.configure(text='%i'%val)
        self.label_gpe_objects_tot_res.configure(fg=color)
    def set_label_gpe_bytes_planner(self, val, color = 'black'):
        self.label_gpe_bytes_planner_res.configure(text='%i'%val)
        self.label_gpe_bytes_planner_res.configure(fg=color)
    def set_label_gpe_objects_planner(self, val, color = 'black'):
        self.label_gpe_objects_planner_res.configure(text='%i'%val)
        self.label_gpe_objects_planner_res.configure(fg=color)
    def set_scrolledtext_global_path(self, txt):
        self.scrolledtext_global_path.delete(1.0,END)
        self.scrolledtext_global_path.insert(INSERT,txt)
    def set_scrolledtext_local_path(self, txt):
        self.scrolledtext_local_path.delete(1.0,END)
        self.scrolledtext_local_path.insert(INSERT,txt)

    def enable_button_global_plan(self):
        self.button_global_plan.configure(state='normal')
    def diable_button_global_plan(self):
        self.button_global_plan.configure(state='disabled')

    def set_label_local_plan_status(self, txt, color = 'black'):
        self.label_local_plan_status_res.configure(text=txt)
        self.label_local_plan_status_res.configure(fg=color)
    def set_label_local_plan_time(self, val, color = 'black'):
        self.label_local_plan_time_res.configure(text='%.02f [s]'%val)
        self.label_local_plan_time_res.configure(fg=color)
    def set_label_local_uav_y(self, val, color = 'black'):
        self.label_local_uav_y_res.configure(text='%.01f [m]'%val)
        self.label_local_uav_y_res.configure(fg=color)
    def set_label_local_uav_x(self, val, color = 'black'):
        self.label_local_uav_x_res.configure(text='%.01f [m]'%val)
        self.label_local_uav_x_res.configure(fg=color)
    def set_label_local_uav_z_rel(self, val, color = 'black'):
        self.label_local_uav_z_rel_res.configure(text='%.01f [m]'%val)
        self.label_local_uav_z_rel_res.configure(fg=color)
    def set_label_local_uav_status(self, txt, color = 'black'):
        self.label_local_uav_status_res.configure(text=txt)
        self.label_local_uav_status_res.configure(fg=color)

    def global_planner_thread(self, point_start, point_goal, path_planner, step_size_horz, step_size_vert, search_time_max):
        if self.parent_class.plan_path_global(point_start, point_goal, path_planner, step_size_horz=step_size_horz, step_size_vert=step_size_vert, search_time_max=search_time_max): # Plan path and test the result to update the GUI
            self.button_local_plan.configure(state='normal')
            self.button_global_plan.configure(state='normal')
            self.button_global_plan.configure(text='Start global planning')
            self.button_show_result_webpage_global.configure(state='normal')
            self.button_evaluate_path.configure(state='normal')
            self.button_web_visualize_global.configure(state='normal')
        else: # The global path planner failed and therefore diable the local path planner and change the option to continue
            self.button_local_plan.configure(state='disabled')
            self.button_global_plan.configure(state='normal')
            self.button_show_result_webpage_global.configure(state='disabled')
            self.button_global_plan.configure(text='Continue global planning')
            self.button_evaluate_path.configure(state='disabled')
            self.button_web_visualize_global.configure(state='disabled')
    def start_global_path_planning(self):
        self.button_global_plan.configure(state='disabled')
        self.button_local_plan.configure(state='disabled')
        self.button_show_result_webpage_global.configure(state='disabled')
        self.button_evaluate_path.configure(state='disabled')
        self.button_web_visualize_global.configure(state='disabled')

        # Get data from the GUI
        path_planner = self.combo_planner_type.get()
        start_point_3dDICT = {'lat': float(self.input_start_point_lat.get()), 'lon': float(self.input_start_point_lon.get()), 'alt_rel': 0}
        goal_point_3dDICT  = {'lat': float(self.input_goal_point_lat.get()), 'lon': float(self.input_goal_point_lon.get()), 'alt_rel': 0}
        step_size_horz = float(self.input_step_size_horz.get())
        step_size_vert = float(self.input_step_size_vert.get())
        search_time_max = float(self.input_search_time_max.get())

        # Create and start the thread
        thread_global_planning = threading.Thread(target=self.global_planner_thread, args=(start_point_3dDICT, goal_point_3dDICT, path_planner, step_size_horz, step_size_vert, search_time_max))
        thread_global_planning.start()
        #thread_global_planning.join()

    def local_planner_thread(self, path_planner, step_size_horz, step_size_vert, max_search_time, time_step, acceleration_factor):
        self.parent_class.plan_path_local(path_planner = path_planner, step_size_horz = step_size_horz, step_size_vert = step_size_vert, time_step = time_step, acceleration_factor = acceleration_factor)
        self.button_global_plan.configure(state='normal')
        self.button_local_plan.configure(state='normal')
        self.button_web_visualize_local.configure(state='normal')
        self.button_show_result_webpage_local.configure(state='normal')
    def start_local_path_planning(self):
        self.button_global_plan.configure(state='disabled')
        self.button_local_plan.configure(state='disabled')
        self.button_web_visualize_local.configure(state='disabled')
        self.button_show_result_webpage_local.configure(state='disabled')
        # Get data from the GUI
        path_planner = str(self.combo_planner_type.get())
        time_step = float(self.input_time_step.get())
        acceleration_factor = float(self.input_acceleration_factor.get())
        step_size_horz = float(self.input_replan_step_size_horz.get())
        step_size_vert = float(self.input_replan_step_size_vert.get())
        search_time_max = float(self.input_replan_search_time_max.get())
        # Create and start the thread
        thread_local_planning = threading.Thread(target=self.local_planner_thread, args=(path_planner, step_size_horz, step_size_vert, search_time_max, time_step, acceleration_factor))
        thread_local_planning.start()

    def show_result_webpage_global(self):
        self.parent_class.draw_planned_path_global()
        self.parent_class.map_plotter_global.show_plot()

    def show_result_webpage_local(self):
        self.parent_class.draw_planned_path_local()
        self.parent_class.map_plotter_local.show_plot()

    def show_web_visualize_global(self):
        route_id = self.parent_class.visualize_path_3d_global()
        if not None:
            url = 'http://uas.heltner.net/routes/'+str(route_id)+'/3d'
            webbrowser.open_new(url)
    def show_web_visualize_local(self):
        route_id = self.parent_class.visualize_path_3d_local()
        if not None:
            url = 'http://uas.heltner.net/routes/'+str(route_id)+'/3d'
            webbrowser.open_new(url)

    def path_evaluation_thread(self):
        self.parent_class.evaluate_path()
    def start_evaluation(self):
        # Create and start the thread
        thread_evaluate_path = threading.Thread(target=self.path_evaluation_thread)
        thread_evaluate_path.start()


    def run(self):
        self.root = Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback_close)
        self.root.title("UAV Path Planner")
        #self.root.geometry('{}x{}'.format(460, 350))

        """ Left side layout """
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
        self.label_start_point_lon = Label(self.root, text="Longitude [dd]:")
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
        self.label_goal_point_lon = Label(self.root, text="Longitude [dd]:")
        self.label_goal_point_lon.grid(row=row_num_left, column=0)
        self.input_goal_point_lon = Entry(self.root,width=10)
        self.input_goal_point_lon.insert(0, self.DEFAULT_GOAL_LON)
        self.input_goal_point_lon.grid(row=row_num_left, column=1)

        row_num_left += 1
        self.label_options  = Label(self.root, text="Options global path planner", font=("Arial Bold", 10))
        self.label_options.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_step_size_horz = Label(self.root, text="Horizontal step-size [m]:")
        self.label_step_size_horz.grid(row=row_num_left, column=0)
        self.input_step_size_horz = Entry(self.root,width=10)
        self.input_step_size_horz.insert(0, self.DEFAULT_STEP_SIZE_HORZ)
        self.input_step_size_horz.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_step_size_vert = Label(self.root, text="Vertical step-size [m]:")
        self.label_step_size_vert.grid(row=row_num_left, column=0)
        self.input_step_size_vert = Entry(self.root,width=10)
        self.input_step_size_vert.insert(0, self.DEFAULT_STEP_SIZE_VERT)
        self.input_step_size_vert.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_search_time_max = Label(self.root, text="Max search time [s]:")
        self.label_search_time_max.grid(row=row_num_left, column=0)
        self.input_search_time_max = Entry(self.root,width=10)
        self.input_search_time_max.insert(0, self.DEFAULT_SEARCH_TIME_MAX)
        self.input_search_time_max.grid(row=row_num_left, column=1)

        row_num_left += 1
        self.button_global_plan = Button(self.root, text="Start global planning", command=self.start_global_path_planning)
        self.button_global_plan.configure(state='disabled')
        self.button_global_plan.grid(row=row_num_left, column=0, columnspan = 2)

        row_num_left += 1
        self.label_options_local  = Label(self.root, text="Options local path planner", font=("Arial Bold", 10))
        self.label_options_local.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_time_step = Label(self.root, text="Time step [s]:")
        self.label_time_step.grid(row=row_num_left, column=0)
        self.input_time_step = Entry(self.root,width=10)
        self.input_time_step.insert(0, self.DEFAULT_TIME_STEP)
        self.input_time_step.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_acceleration_factor = Label(self.root, text="Playback speed:")
        self.label_acceleration_factor.grid(row=row_num_left, column=0)
        self.input_acceleration_factor = Entry(self.root,width=10)
        self.input_acceleration_factor.insert(0, self.DEFAULT_ACCELERATION_FACTOR)
        self.input_acceleration_factor.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_replan_step_size_horz = Label(self.root, text="Replan horizontal step-size [m]:")
        self.label_replan_step_size_horz.grid(row=row_num_left, column=0)
        self.input_replan_step_size_horz = Entry(self.root,width=10)
        self.input_replan_step_size_horz.insert(0, self.DEFAULT_STEP_SIZE_HORZ_LOCAL)
        self.input_replan_step_size_horz.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_replan_step_size_vert = Label(self.root, text="Replan vertical step-size [m]:")
        self.label_replan_step_size_vert.grid(row=row_num_left, column=0)
        self.input_replan_step_size_vert = Entry(self.root,width=10)
        self.input_replan_step_size_vert.insert(0, self.DEFAULT_STEP_SIZE_VERT_LOCAL)
        self.input_replan_step_size_vert.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_replan_search_time_max = Label(self.root, text="Replan max search time [s]:")
        self.label_replan_search_time_max.grid(row=row_num_left, column=0)
        self.input_replan_search_time_max = Entry(self.root,width=10)
        self.input_replan_search_time_max.insert(0, self.DEFAULT_SEARCH_TIME_MAX_LOCAL)
        self.input_replan_search_time_max.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.button_local_plan = Button(self.root, text="Start local planning", command=self.start_local_path_planning)
        self.button_local_plan.configure(state='disabled') # disable the button since it cannot make a local plan before it has made a global plan
        self.button_local_plan.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_sim_info = Label(self.root, text="Simulation information", font=("Arial Bold", 12))
        self.label_sim_info.grid(row=row_num_left, column=0, columnspan = 2)
        row_num_left += 1
        self.label_local_plan_status = Label(self.root, text="Status:")
        self.label_local_plan_status.grid(row=row_num_left, column=0)
        self.label_local_plan_status_res = Label(self.root, text="idle")
        self.label_local_plan_status_res.configure(fg='green')
        self.label_local_plan_status_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_local_plan_time = Label(self.root, text="Time:")
        self.label_local_plan_time.grid(row=row_num_left, column=0)
        self.label_local_plan_time_res = Label(self.root, text="N/A")
        self.label_local_plan_time_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_local_uav_y = Label(self.root, text="UAV y:")
        self.label_local_uav_y.grid(row=row_num_left, column=0)
        self.label_local_uav_y_res = Label(self.root, text="N/A")
        self.label_local_uav_y_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_local_uav_x = Label(self.root, text="UAV x:")
        self.label_local_uav_x.grid(row=row_num_left, column=0)
        self.label_local_uav_x_res = Label(self.root, text="N/A")
        self.label_local_uav_x_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_local_uav_z_rel = Label(self.root, text="UAV z_rel:")
        self.label_local_uav_z_rel.grid(row=row_num_left, column=0)
        self.label_local_uav_z_rel_res = Label(self.root, text="N/A")
        self.label_local_uav_z_rel_res.grid(row=row_num_left, column=1)
        row_num_left += 1
        self.label_local_uav_status = Label(self.root, text="UAV status:")
        self.label_local_uav_status.grid(row=row_num_left, column=0)
        self.label_local_uav_status_res = Label(self.root, text="N/A")
        self.label_local_uav_status_res.grid(row=row_num_left, column=1)

        """ Right side layout """
        row_num_right = 0
        self.label_data_sources = Label(self.root, text="Data data sources", font=("Arial Bold", 12))
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
        row_num_right += 1
        self.label_data_rally_point = Label(self.root, text="Rally points:")
        self.label_data_rally_point.grid(row=row_num_right, column=2)
        self.label_data_rally_point_res = Label(self.root, text="not loaded")
        self.label_data_rally_point_res.configure(fg='red')
        self.label_data_rally_point_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        # gpe = global path evaluator
        self.label_gpe = Label(self.root, text="Global path evaluator", font=("Arial Bold", 12))
        self.label_gpe.grid(row=row_num_right, column=2, columnspan = 2)
        row_num_right += 1
        self.button_evaluate_path = Button(self.root, text="Evaluate", command=self.start_evaluation)
        self.button_evaluate_path.configure(state='disabled') # Disabled because no global plan has been made
        self.button_evaluate_path.grid(row=row_num_right, column=2, columnspan = 2)
        row_num_right += 1
        self.label_gpe_fitness = Label(self.root, text="Fitness:")
        self.label_gpe_fitness.grid(row=row_num_right, column=2)
        self.label_gpe_fitness_res = Label(self.root, text="N/A")
        self.label_gpe_fitness_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_dist_tot = Label(self.root, text="Total distance:")
        self.label_gpe_dist_tot.grid(row=row_num_right, column=2)
        self.label_gpe_dist_tot_res = Label(self.root, text="N/A")
        self.label_gpe_dist_tot_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_dist_horz = Label(self.root, text="Horizontal distance:")
        self.label_gpe_dist_horz.grid(row=row_num_right, column=2)
        self.label_gpe_dist_horz_res = Label(self.root, text="N/A")
        self.label_gpe_dist_horz_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_dist_vert = Label(self.root, text="Vertical distance:")
        self.label_gpe_dist_vert.grid(row=row_num_right, column=2)
        self.label_gpe_dist_vert_res = Label(self.root, text="N/A")
        self.label_gpe_dist_vert_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_eta = Label(self.root, text="Estimated flight time:")
        self.label_gpe_eta.grid(row=row_num_right, column=2)
        self.label_gpe_eta_res = Label(self.root, text="N/A")
        self.label_gpe_eta_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_wps = Label(self.root, text="Waypoints:")
        self.label_gpe_wps.grid(row=row_num_right, column=2)
        self.label_gpe_wps_res = Label(self.root, text="N/A")
        self.label_gpe_wps_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_runtime = Label(self.root, text="Runtime:")
        self.label_gpe_runtime.grid(row=row_num_right, column=2)
        self.label_gpe_runtime_res = Label(self.root, text="N/A")
        self.label_gpe_runtime_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_bytes_tot = Label(self.root, text="Bytes total:")
        self.label_gpe_bytes_tot.grid(row=row_num_right, column=2)
        self.label_gpe_bytes_tot_res = Label(self.root, text="N/A")
        self.label_gpe_bytes_tot_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_objects_tot = Label(self.root, text="Objects total:")
        self.label_gpe_objects_tot.grid(row=row_num_right, column=2)
        self.label_gpe_objects_tot_res = Label(self.root, text="N/A")
        self.label_gpe_objects_tot_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_bytes_planner = Label(self.root, text="Bytes planner:")
        self.label_gpe_bytes_planner.grid(row=row_num_right, column=2)
        self.label_gpe_bytes_planner_res = Label(self.root, text="N/A")
        self.label_gpe_bytes_planner_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_gpe_objects_planner = Label(self.root, text="Objects planner:")
        self.label_gpe_objects_planner.grid(row=row_num_right, column=2)
        self.label_gpe_objects_planner_res = Label(self.root, text="N/A")
        self.label_gpe_objects_planner_res.grid(row=row_num_right, column=3)

        row_num_right += 1
        self.label_planning_info = Label(self.root, text="Planning information", font=("Arial Bold", 12))
        self.label_planning_info.grid(row=row_num_right, column=2, columnspan = 2)
        row_num_right += 1
        self.label_global_plan_status = Label(self.root, text="Status:")
        self.label_global_plan_status.grid(row=row_num_right, column=2)
        self.label_global_plan_status_res = Label(self.root, text="idle")
        self.label_global_plan_status_res.configure(fg='green')
        self.label_global_plan_status_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_start_heuristic = Label(self.root, text="Start heuristic:")
        self.label_global_plan_start_heuristic.grid(row=row_num_right, column=2)
        self.label_global_plan_start_heuristic_res = Label(self.root, text="N/A")
        self.label_global_plan_start_heuristic_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_cur_heuristic = Label(self.root, text="Best heuristic:")
        self.label_global_plan_cur_heuristic.grid(row=row_num_right, column=2)
        self.label_global_plan_cur_heuristic_res = Label(self.root, text="N/A")
        self.label_global_plan_cur_heuristic_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_horz_step_size = Label(self.root, text="Horizontal step-size:")
        self.label_global_plan_horz_step_size.grid(row=row_num_right, column=2)
        self.label_global_plan_horz_step_size_res = Label(self.root, text="N/A")
        self.label_global_plan_horz_step_size_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_vert_step_size = Label(self.root, text="Vertical step-size:")
        self.label_global_plan_vert_step_size.grid(row=row_num_right, column=2)
        self.label_global_plan_vert_step_size_res = Label(self.root, text="N/A")
        self.label_global_plan_vert_step_size_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_search_time = Label(self.root, text="Search time:")
        self.label_global_plan_search_time.grid(row=row_num_right, column=2)
        self.label_global_plan_search_time_res = Label(self.root, text="N/A")
        self.label_global_plan_search_time_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_nodes_visited = Label(self.root, text="Nodes visited:")
        self.label_global_plan_nodes_visited.grid(row=row_num_right, column=2)
        self.label_global_plan_nodes_visited_res = Label(self.root, text="N/A")
        self.label_global_plan_nodes_visited_res.grid(row=row_num_right, column=3)
        row_num_right += 1
        self.label_global_plan_nodes_explored = Label(self.root, text="Nodes explored:")
        self.label_global_plan_nodes_explored.grid(row=row_num_right, column=2)
        self.label_global_plan_nodes_explored_res = Label(self.root, text="N/A")
        self.label_global_plan_nodes_explored_res.grid(row=row_num_right, column=3)

        """ Both sides """
        if row_num_left > row_num_right:
            row_num_both = row_num_left
        else:
            row_num_both = row_num_right
        row_num_both += 1
        self.label_global_path = Label(self.root, text="Global path:")
        self.label_global_path.grid(row=row_num_both, column=0)
        self.scrolledtext_global_path = tkst.ScrolledText(self.root,height=10)
        self.scrolledtext_global_path.grid(row=row_num_both, column=1, columnspan=3)

        row_num_both += 1
        self.label_local_path = Label(self.root, text="Local path:")
        self.label_local_path.grid(row=row_num_both, column=0)
        self.scrolledtext_local_path = tkst.ScrolledText(self.root,height=10)
        self.scrolledtext_local_path.grid(row=row_num_both, column=1, columnspan=3)

        row_num_both += 1
        self.button_show_result_webpage_global = Button(self.root, text="2D global path visualization (local)", command=self.show_result_webpage_global)
        self.button_show_result_webpage_global.configure(state='disabled') # Disabled because no global plan has been made
        self.button_show_result_webpage_global.grid(row=row_num_both, column=0, columnspan = 2)
        #row_num_both += 1
        self.button_web_visualize_global = Button(self.root, text="3D global path visualization (online)", command=self.show_web_visualize_global)
        self.button_web_visualize_global.configure(state='disabled') # Disabled because no global plan has been made
        self.button_web_visualize_global.grid(row=row_num_both, column=2, columnspan = 2)

        row_num_both += 1
        self.button_show_result_webpage_local = Button(self.root, text="2D local path visualization (local)", command=self.show_result_webpage_local)
        self.button_show_result_webpage_local.configure(state='disabled') # Disabled because no global plan has been made
        self.button_show_result_webpage_local.grid(row=row_num_both, column=0, columnspan = 2)
        #row_num_both += 1
        self.button_web_visualize_local = Button(self.root, text="3D local path visualization (online)", command=self.show_web_visualize_local)
        self.button_web_visualize_local.configure(state='disabled') # Disabled because no global plan has been made
        self.button_web_visualize_local.grid(row=row_num_both, column=2, columnspan = 2)




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
