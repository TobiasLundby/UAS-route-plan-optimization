#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-04-10 TL First version
"""

"""
Description:
Path planner for UAVs along with a path evaluator (fitness etc.)
License: BSD 3-Clause
"""

""" Import libraries """
from math import pow, sqrt, pi, cos
from termcolor import colored
from pyproj import Geod
from copy import copy, deepcopy
import csv # for saving statistics and path
from heapq import * # for the heap used in the A star algorithm
from libs.coordinate import coordinate_transform
from libs.map_plotter import map_plotter
from libs.memory_usage import memory_usage
from data_sources.no_fly_zones.kml_reader import kml_no_fly_zones_parser
from data_sources.drone_id.get_droneid_data import droneid_data
from data_sources.height.srtm import srtm_lib
from libs.various import *
from libs.gui import path_planner_gui
from shapely import geometry # used to calculate the distance to polygons
from rdp import rdp
import logging
import time # used for sleeping the main loop

""" User constants """
DRONEID_FORCE_ALL_REAL  = True

class UAV_path_planner():
    """ User constants """
    PRINT_STATISTICS        = True
    SAVE_STATISTICS_TO_FILE = True
    """ UAV constants """
    UAV_NOMINAL_AIRSPEED_HORZ_MPS   = 15 # unit: m/s
    UAV_NOMINAL_AIRSPEED_VERT_MPS   = 5  # unit: m/s
    UAV_NOMINAL_BATTERY_TIME_MIN    = 20 # unit: min
    UAV_NOMINAL_BATTERY_TIME_S      = UAV_NOMINAL_BATTERY_TIME_MIN*60 # unit: s
    """ Legislation constants """
    MAX_FLYING_ALTITUDE             = 100 # unit: m; max flying altitude for rural areas
    """ Path planning constants """
    IDEAL_FLIGHT_ALTITUDE           = 30 # unit: m
    PATH_PLANNER_ASTAR              = 0 # just an internal number to recognize the path planner
    PATH_PLANNER_NAMES              = ['A star algorithm', 'RRT']
    #PATH_PLANNER                    = PATH_PLANNER_ASTAR # the name of the chosen path path planner; NOTE the user can set this
    PATH_PLANNER_SPACE_DIMENSIONS   = 3 # NOTE that this only space dimension and the time dimension is therefore not included
    PP_PRINT_POPPED_NODE            = False
    PP_PRINT_EXPLORE_NODE           = False
    DRAW_OPEN_LIST                  = False
    DRAW_CLOSED_LIST                = False
    GLOBAL_PLANNING                 = 0
    LOCAL_PLANNING                  = 1
    EMERGENCY_PLANNING              = 2
    """ Path evaluation (path fitness) factor constants """
    HORZ_DISTANCE_FACTOR            = 1 # unit: unitless
    VERT_DISTANCE_FACTOR            = 2 # unit: unitless
    TRAVEL_TIME_FACTOR              = 1 # unit: unitless
    WAYPOINTS_FACTOR                = 1 # unit: unitless
    AVG_WAYPOINT_DIST_FACTOR        = 1 # unit: unitless
    """ Other constants """
    NO_FLY_ZONE_HEIGHT              = 100 # unit: m
    NO_FLY_ZONE_BASE_BUFFER_DIST    = 10 # unit: m; added to the buffer distance if the funcitons are not explicitly told not to
    NO_FLY_ZONE_REDUCE_FLIGHT_DISTANCE_FACTOR = 2 # unit: unitless ; used for reducing the no-fly zones based on the flight distance of the start point, reason: if the wind is 12m/s and the drone if flying with 15m/s the safe reducing is 1.8 (not safe) ≈ 2 (safe)
    GEOID_WGS84                     = Geod(ellps='WGS84') # used for calculating Great Circle Distance

    def __init__(self, debug = False, data_path_no_fly_zones = None, data_path_height_map = None):
        """ Constructor """
        self.debug = debug
        self.debug_test = False

        # Instantiate memory_usage class
        self.memory_usage_module = memory_usage()
        self.memory_usage_module.save_heap_start() # Save the heap size before anything else

        # Instantiate coordinate transform class
        self.coord_conv = coordinate_transform(debug = False)

        # Instantiate map plotter class
        self.map_plotter = map_plotter(self.coord_conv, debug = False)

        # Instantiate and start the GUI
        self.gui = path_planner_gui(self)
        self.gui.start()

        # Set initial values for loading of no-fly zones
        self.no_fly_zones_loaded = False
        self.no_fly_zone_reader_module = None
        self.no_fly_zone_polygons = []
        self.no_fly_zone_polygons_reduced = []
        self.no_fly_zones_reduced = False
        # Instantiate and load no-fly zone class
        self.gui.on_main_thread( lambda: self.gui.set_label_no_fly_zones('loading', 'yellow') )
        no_fly_zone_load_res = False
        if data_path_no_fly_zones == None: no_fly_zone_load_res = self.no_fly_zone_init_and_parse()
        else: no_fly_zone_load_res = self.no_fly_zone_init_and_parse(data_path_no_fly_zones)
        if no_fly_zone_load_res: self.gui.on_main_thread( lambda: self.gui.set_label_no_fly_zones('loaded', 'green') )
        else: self.gui.on_main_thread( lambda: self.gui.set_label_no_fly_zones('error', 'red') )
        # Reduce no-fly zones
        if self.reduce_ploygons_geodetic(start_point_3dDICT, goal_point_3dDICT): self.gui.on_main_thread( lambda: self.gui.set_label_no_fly_zones('loaded, reduced', 'green') )
        else: self.gui.on_main_thread( lambda: self.gui.set_label_no_fly_zones('loaded, not reduced', 'yellow') )

        # Set initial values for height map
        self.height_map_loaded = False
        # Instantiate and load height map class
        if data_path_height_map == None:
            self.gui.on_main_thread( lambda: self.gui.set_label_height_map('missing', 'red') )
        else:
            self.gui.on_main_thread( lambda: self.gui.set_label_height_map('loading', 'yellow') )
            self.height_map_init('data_sources/height/', start_point_3dDICT)
            self.gui.on_main_thread( lambda: self.gui.set_label_height_map('loaded', 'green') )

        # Instantiate and load droneid data class
        self.droneid = droneid_data(debug = False, force_sim_to_real = DRONEID_FORCE_ALL_REAL)
        self.gui.on_main_thread( lambda: self.gui.set_label_drone_id('loading', 'yellow') )
        if self.droneid.download_data():
            self.gui.on_main_thread( lambda: self.gui.set_label_drone_id('loaded', 'green') )
        else:
            self.gui.on_main_thread( lambda: self.gui.set_label_drone_id('error', 'red') )

        # Set initial values for rally points
        self.rally_points_loaded = False

        # Initialize objects for prevoius A start path plannings
        self.prev_Astar_step_size_and_points = []
        self.prev_Astar_closed_lists = []
        self.prev_Astar_open_lists = []
        self.prev_Astar_came_froms = []
        self.prev_Astar_g_scores = []
        self.prev_Astar_f_scores = []
        self.prev_Astar_smallest_heuristics = []

    def stop_gui(self):
        self.gui.stop_thread()

    """ Height map """
    def height_map_init(self, dir, start_point2d):
        """
        Instantiates the height map module and tests it using a point so the data is downloaded (if needed)
        Input: filename of height map root and a test point (preferrably the start point)
        Output: none
        """
        # Instantiate height map class
        self.height_map = srtm_lib(dir)
        if isinstance(start_point2d, dict):
            test_point2d_geodetic = [start_point2d['lat'], start_point2d['lon']]
        else:
            test_point2d_geodetic = start_point2d
        self.height_map.get_elevation(test_point2d_geodetic[0], test_point2d_geodetic[1])
        self.height_map_loaded = True

    def get_altitude_geodetic(self, test_point2d):
        """
        Returns the altitude/height in m above sea level
        Input: geodetic 2d test point (can be more dims but these are ignored)
        Output: height (if data void -32768)
        """
        if self.height_map_loaded:
            if isinstance(test_point2d, dict):
                test_point2d_geodetic = [test_point2d['lat'], test_point2d['lon']]
            else:
                test_point2d_geodetic = test_point2d
            return self.height_map.get_elevation(test_point2d_geodetic[0], test_point2d_geodetic[1])
        else:
            print colored('Height map not loaded', TERM_COLOR_ERROR)
            return DATA_VOID

    def get_altitude_UTM(self, test_point2d):
        """
        Returns the altitude/height in m above sea level
        Input: UTM 2d test point (can be more dims but these are ignored)
        Output: height (if data void -32768)
        """
        if isinstance(test_point2d, dict):
            test_point2d_UTM = [test_point2d['hemisphere'], test_point2d['zone'], test_point2d['y'], test_point2d['x']]
        else:
            test_point2d_UTM = [test_point2d[4], test_point2d[5], test_point2d[0], test_point2d[1]]
        test_point2d_geodetic = self.coord_conv.UTM2geodetic(test_point2d_UTM[0], test_point2d_UTM[1], test_point2d_UTM[2], test_point2d_UTM[3])
        return self.get_altitude_geodetic(test_point2d_geodetic)

    """ No-fly zones """
    def no_fly_zone_init_and_parse(self, file_name_in = None):
        """
        Loads and parses the KML file provided by the filename using the kml_no_fly_zones_parser class
        Input: filename of KML file to parse (if not provided a download is initiated)
        Output: loaded status (bool: True = loaded, False = not loaded)
        """
        self.no_fly_zone_reader_module = kml_no_fly_zones_parser()
        if file_name_in == None:
            self.no_fly_zones_loaded = self.no_fly_zone_reader_module.download_and_parse_data()
        else:
            self.no_fly_zones_loaded = self.no_fly_zone_reader_module.parse_file(file_name_in)

        polygons_loaded = self.load_polygons()

        return (self.no_fly_zones_loaded and polygons_loaded)

    def load_polygons(self):
        """
        Loads the polygons into the combined polygon object (no_fly_zone_polygons)
        Input: none
        Output: loaded status (bool: True = loaded, False = not loaded)
        """
        if self.no_fly_zones_loaded:
            no_fly_zones = self.no_fly_zone_reader_module.get_zones() # extract the no-fly zones
            for no_fly_zone in no_fly_zones:
                no_fly_zone_coordinates = no_fly_zone['coordinates'] # extract the coordinates
                no_fly_zone_coordinates_UTM = self.coord_conv.pos3d_geodetic2pos3d_UTM_multiple(no_fly_zone_coordinates) # convert coordinates to UTM
                no_fly_zone_coordinates_UTM_y_x = [[x[0],x[1]] for x in no_fly_zone_coordinates_UTM] # extract y and x
                self.no_fly_zone_polygons.append(geometry.Polygon(no_fly_zone_coordinates_UTM_y_x)) # append the polygon to the combined polygons
            return self.test_polygons() # return combined polygons

    def test_polygons(self):
        """
        Tests the polygon implementation with 2 known points, one that are within the polygon and one that isn't.
        It finds a specific polygon (ID: 01c2f279-fc2c-421d-baa7-60afd0a0b8ac) and uses predefined points for testing
        Note that it required the specific polygon to be in the polygon list
        Input: none
        Output: bool (True = polygons work, False = polygons does not work)
        """
        res_bool, index = self.no_fly_zone_reader_module.get_polygon_index_from_id('01c2f279-fc2c-421d-baa7-60afd0a0b8ac')
        if res_bool:
            # Calcluate dist for point 1 which is inside the polygon
            test_point_geodetic = [55.399512, 10.319328, 0]
            test_point_UTM = self.coord_conv.pos3d_geodetic2pos3d_UTM(test_point_geodetic)
            dist1 = self.no_fly_zone_polygons[index].distance(geometry.Point(test_point_UTM))
            if self.debug_test:
                print '\nTest point 2: should be inside: distance = %.02f' % dist1
            # Calcluate dist for point 2 which is NOT inside the polygon
            test_point_geodetic = [55.397001, 10.307698, 0]
            test_point_UTM = self.coord_conv.pos3d_geodetic2pos3d_UTM(test_point_geodetic)
            dist2 = self.no_fly_zone_polygons[index].distance(geometry.Point(test_point_UTM))
            if self.debug_test:
                print '\nTest point 2: should NOT be inside: distance = %.02f' % dist2
            if dist1 == 0.0 and dist2 >= 200:
                return True
            else:
                return False
        else:
            if self.debug_test:
                print '\nSpecific no-fly zone for test is not in list, skipping test'
            return True

    def reduce_ploygons_geodetic(self, point3d_start_geodetic, point3d_goal_geodetic):
        """
        Reduce the no-fly_zones / polygons based on the start point and end point
        Input: start and goal point in geodetic format
        Output: bool (True = reduced, False = not reduced)
        """
        start_point_3dDICT = {'lat': 55.431122, 'lon': 10.420436, 'alt_rel': 0}
        # Test format and convert points
        if isinstance(point3d_start_geodetic, dict):
            test_point3d_start_geodetic = [point3d_start_geodetic['lat'], point3d_start_geodetic['lon'], point3d_start_geodetic['alt_rel']]
        else:
            test_point3d_start_geodetic = point3d_start_geodetic
        test_point_start_UTM = self.coord_conv.pos3d_geodetic2pos3d_UTM(test_point3d_start_geodetic)
        if isinstance(point3d_start_geodetic, dict):
            test_point3d_goal_geodetic = [point3d_goal_geodetic['lat'], point3d_goal_geodetic['lon'], point3d_goal_geodetic['alt_rel']]
        else:
            test_point3d_goal_geodetic = point3d_goal_geodetic
        test_point_goal_UTM = self.coord_conv.pos3d_geodetic2pos3d_UTM(test_point3d_goal_geodetic)

        return self.reduce_ploygons_UTM(test_point_start_UTM, test_point_goal_UTM)

    def reduce_ploygons_UTM(self, point3d_start_UTM, point3d_goal_UTM):
        """
        Reduce the no-fly_zones / polygons based on the start point and end point
        Input: start and goal point in UTM format
        Output: bool (True = reduced, False = not reduced)
        """
        if isinstance(point3d_start_UTM, dict):
            test_point3d_start_UTM = [point3d_start_UTM['y'], point3d_start_UTM['x'], point3d_start_UTM['z_rel']]
        else:
            test_point3d_start_UTM = point3d_start_UTM
        if isinstance(point3d_start_UTM, dict):
            test_point3d_goal_UTM = [point3d_goal_UTM['y'], point3d_goal_UTM['x'], point3d_goal_UTM['z_rel']]
        else:
            test_point3d_goal_UTM = point3d_start_UTM
        #print test_point3d_start_UTM
        #print test_point3d_goal_UTM

        self.no_fly_zone_polygons_reduced = []

        # Calculate nominal max flight time
        max_flight_distance_scaled = self.UAV_NOMINAL_BATTERY_TIME_S * self.UAV_NOMINAL_AIRSPEED_HORZ_MPS * self.NO_FLY_ZONE_REDUCE_FLIGHT_DISTANCE_FACTOR
        if self.debug:
            print colored(SUB_RES_INDENT+'Max flight distance scaled %.02f [m] = %.02f [km]' % (max_flight_distance_scaled, max_flight_distance_scaled/1000), TERM_COLOR_RES)

        for i in range(len(self.no_fly_zone_polygons)):
            result_start_point, dist_start_point = self.is_point_in_no_fly_zone_UTM(test_point3d_start_UTM, i, buffer_distance_m = 0, use_base_buffer_distance_m = False)
            if dist_start_point <= max_flight_distance_scaled:
                self.no_fly_zone_polygons_reduced.append(self.no_fly_zone_polygons[i])

        if self.debug:
            print colored(SUB_RES_INDENT+'No-fly zones / polygons reduced from %i to %i' % (len(self.no_fly_zone_polygons), len(self.no_fly_zone_polygons_reduced)), TERM_COLOR_RES)

        if len(self.no_fly_zone_polygons) > len(self.no_fly_zone_polygons_reduced):
            self.no_fly_zones_reduced = True
            return True
        return False

    def is_point_in_no_fly_zones_UTM(self, point3dUTM, buffer_distance_m = 0, use_base_buffer_distance_m = True):
        """
        Tests if an UTM point is within any of the no-fly zone polygons
        Input: UTM point3d as array (y, x, alt_rel ...) along with an optional buffer distance [m] and use of base buffer distance (define)
        Ouput: bool (True: point is inside) and distance to nearest polygon [m]
        """
        smallest_dist = INF
        if self.no_fly_zones_reduced:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons_reduced
        else:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons
        for i in range(len(no_fly_zone_polygons_to_use)):
            within, dist = self.is_point_in_no_fly_zone_UTM(point3dUTM, i, buffer_distance_m, use_base_buffer_distance_m)
            if dist < smallest_dist:
                smallest_dist = dist
            if within:
                return True, dist
        return False, smallest_dist
    def is_point_in_no_fly_zone_UTM(self, point3dUTM, polygon_index, buffer_distance_m = 0, use_base_buffer_distance_m = True):
        """
        Tests if an UTM point is within a specified polygon
        Input: UTM point3d as array (y, x, alt_rel ...) and polygon index along with an optional buffer distance [m] and use of base buffer distance (define)
        Ouput: bool (True: point is inside) and distance to polygon [m]
        """
        if isinstance(point3dUTM, dict):
            test_point3dUTM = [point3dUTM['y'], point3dUTM['x'], point3dUTM['z_rel']]
        else:
            test_point3dUTM = point3dUTM

        if self.no_fly_zones_reduced:
            dist = self.no_fly_zone_polygons_reduced[polygon_index].distance(geometry.Point(test_point3dUTM))
        else:
            dist = self.no_fly_zone_polygons[polygon_index].distance(geometry.Point(test_point3dUTM))

        if use_base_buffer_distance_m:
            if dist <= buffer_distance_m+self.NO_FLY_ZONE_BASE_BUFFER_DIST and test_point3dUTM[2] <= self.NO_FLY_ZONE_HEIGHT+self.NO_FLY_ZONE_BASE_BUFFER_DIST :
                return True, dist
            else:
                return False, dist
        else:
            if dist <= buffer_distance_m and test_point3dUTM[2] <= self.NO_FLY_ZONE_HEIGHT:
                return True, dist
            else:
                return False, dist

    def is_point_in_no_fly_zones_geodetic(self, point3d_geodetic, buffer_distance_m = 0, use_base_buffer_distance_m = True):
        """
        Tests if a geodetic point is within any of the no-fly zone polygons
        Input: geodetic point3d as array (y, x, alt_rel ...) along with an optional buffer distance [m] and use of base buffer distance (define)
        Ouput: bool (True: point is inside) and distance to nearest polygon [m]
        """
        smallest_dist = INF
        if self.no_fly_zones_reduced:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons_reduced
        else:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons
        for i in range(len(no_fly_zone_polygons_to_use)):
            within, dist = self.is_point_in_no_fly_zone_geodetic(point3d_geodetic, i, use_buffer_zone, use_base_buffer_distance_m)
            if dist < smallest_dist:
                smallest_dist = dist
            if within:
                return True, dist
        return False, smallest_dist
    def is_point_in_no_fly_zone_geodetic(self, point3d_geodetic, polygon_index, buffer_distance_m = 0, use_base_buffer_distance_m = True):
        """
        Tests if a geodetic point is within a specified polygon
        Input: geodetic point3d as array (lat, lon, alt_rel) and polygon index along with an optional buffer distance [m] and use of base buffer distance (define)
        Ouput: bool (True: point is inside) and distance to polygon [m]
        """
        if isinstance(point3d_geodetic, dict):
            test_point3d_geodetic = [point3d_geodetic['lat'], point3d_geodetic['lon'], point3d_geodetic['z_rel']]
        else:
            test_point3d_geodetic = point3d_geodetic
        test_point_UTM = self.pos3d_geodetic2pos3d_UTM(test_point3d_geodetic)
        return self.is_point_in_no_fly_zone_UTM(test_point_UTM, polygon_index, use_buffer_zone, use_base_buffer_distance_m)

    """ Path planning functions """
    def plan_path_global(self, point_start, point_goal, path_planner = 0):
        """
        Framework for different path planning algorithms
        Input: start and goal/end point (latitude, longitude in EPSG:4326, and relative altitude) as 3 value array or DICT (lat, lon, alt_rel)
        Output: bool but the path is saved as internal object (planned path of geodetic points (latitude, longitude in EPSG:4326, relative altitude, and relative time))
        """
        self.time_planning_start_s = get_cur_time_epoch() # Save start timestamp
        print colored('\nPath planning started', TERM_COLOR_INFO)
        self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('started', 'yellow') )

        # Convert to pos3dDICT
        try:
            tmp_var = point_start['lat']
        except TypeError:
            point_start_converted_geodetic = self.coord_conv.pos3d2pos3dDICT_geodetic(point_start)
        else:
            point_start_converted_geodetic = point_start
        try:
            tmp_var = point_goal['lat']
        except TypeError:
            point_goal_converted_geodetic = self.coord_conv.pos3d2pos3dDICT_geodetic(point_goal)
        else:
            point_goal_converted_geodetic = point_goal
        if self.debug:
            print colored(INFO_ALT_INDENT+'Start point: lat: %.03f, lon: %.03f' % (point_start_converted_geodetic['lat'], point_start_converted_geodetic['lon']), TERM_COLOR_INFO_ALT)
            print colored(INFO_ALT_INDENT+' Goal point: lat: %.03f, lon: %.03f' % (point_goal_converted_geodetic['lat'], point_goal_converted_geodetic['lon']), TERM_COLOR_INFO_ALT)

        self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('converting points', 'yellow') )
        # Convert to UTM for path planning
        point_start_converted_UTM = self.coord_conv.pos3dDICT_geodetic2pos4dDICT_UTM(point_start_converted_geodetic)
        point_goal_converted_UTM = self.coord_conv.pos3dDICT_geodetic2pos4dDICT_UTM(point_goal_converted_geodetic)
        if self.debug:
            print colored(RES_INDENT+'Start point: %d %c %.5fe %.5fn' % (point_start_converted_UTM['zone'], point_start_converted_UTM['letter'], point_start_converted_UTM['y'], point_start_converted_UTM['x']), TERM_COLOR_RES)
            print colored(RES_INDENT+' Goal point: %d %c %.5fe %.5fn' % (point_goal_converted_UTM['zone'], point_goal_converted_UTM['letter'], point_goal_converted_UTM['y'], point_goal_converted_UTM['x']), TERM_COLOR_RES)

        # Update the start heuristic in the GUI
        self.gui.on_main_thread( lambda: self.gui.set_global_plan_start_heuristic(self.heuristic_a_star(point_start_converted_UTM, point_goal_converted_UTM, self.GLOBAL_PLANNING)) )

        step_size_horz = 75
        step_size_vert = 10
        search_time_max = 60
        # Pre path planning check
        self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('pre-plan check', 'yellow') )
        if not self.pre_planner_check(point_start_converted_UTM, point_goal_converted_UTM, step_size_horz):
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('pre-plan check failed', 'red') )
            print colored('Error: did not pass pre path planning check', TERM_COLOR_ERROR)
            return []

        # Go to selected path planner
        if path_planner == self.PATH_PLANNER_NAMES[0]: # A star path planning algorithm
            #print colored('Planning using A start', TERM_COLOR_INFO)
            #path_UTM = self.plan_planner_Astar(point_start_converted_UTM, point_goal_converted_UTM, 16, 15, type_of_planning = self.GLOBAL_PLANNING, search_time_max = FOREVER)
            #path_UTM = self.plan_planner_Astar(point_start_converted_UTM, point_goal_converted_UTM, 8, 10, type_of_planning = self.GLOBAL_PLANNING, search_time_max = 16)
            path_UTM = self.plan_planner_Astar(point_start_converted_UTM, point_goal_converted_UTM, step_size_horz, step_size_vert, type_of_planning = self.GLOBAL_PLANNING, search_time_max = search_time_max)
            #path_UTM = self.plan_planner_Astar(point_start_converted_UTM, point_goal_converted_UTM, 2, 2.5, type_of_planning = self.GLOBAL_PLANNING, search_time_max = 180)
        else:
            print colored('Path planner type not defined', TERM_COLOR_ERROR)
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('planner not defined', 'red') )
            return []

        # Check if the path planner produced a result
        if len(path_UTM) <= 1:
            print colored('The chosen path planner could not find a solution to the problem', TERM_COLOR_ERROR)
            return False
        # Convert path from UTM to geodetic
        path_geodetic = []
        for i in range(len(path_UTM)):
            path_geodetic.append( self.coord_conv.pos4dDICT_UTM2pos4dDICT_geodetic( path_UTM[i] ) )
        self.time_planning_completed_s = get_cur_time_epoch() # Save completed timestamp

        self.send_path_to_gui_geodetic(path_geodetic)

        self.planned_path_global_UTM = path_UTM
        self.planned_path_global_geodetic = path_geodetic
        self.point_start_global_geodetic = point_start
        self.point_goal_global_geodetic  = point_goal
        return True

    def send_path_to_gui_geodetic(self, path):
        out_str = ''
        for i in range(len(path)):
            out_str += 'Waypoint %d: ' % i
            out_str += str(path[i])
            out_str += '\n'
        self.gui.on_main_thread( lambda: self.gui.set_scrolledtext_global_path(out_str) )

    def pre_planner_check(self, point_start_UTM, point_goal_UTM, step_size_horz = 0):
        """
        Pre path planning check
        Checks:

        Input: start and goal point in UTM format
        Output: bool (True = passed, False = Failed)
        """
        if self.is_point_in_no_fly_zones_UTM(point_start_UTM, step_size_horz)[0]:
            print colored('Start point is inside geofence', TERM_COLOR_ERROR)
            return False
        elif self.is_point_in_no_fly_zones_UTM(point_goal_UTM, step_size_horz)[0]:
            print colored('Goal point is inside geofence', TERM_COLOR_ERROR)
            return False
        # Check if the goal is rechable with the drone specifications - TODO - remember to include the wind speed and direction
        # Check if the weather exceeds limits - TODO
        return True

    def plan_planner_Astar(self, point_start, point_goal, step_size_horz, step_size_vert, type_of_planning = 0, max_node_exploration = INF, search_time_max = FOREVER, force_replanning = False):
        """
        A star algorithm path planner
        Input: start and goal point in UTM format
                optional type of planning (see node cost)
                optional maximum nodes to explore
                optional maximum search time
                optional force replanning if it can continue on existing data
        Output: planned path of UTM points
        Inspiration:
            Wikipedia: https://en.wikipedia.org/wiki/A*_search_algorithm
            Christian Careaga: http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/
            Python docs: https://docs.python.org/2/library/heapq.html
        """
        self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('initializing A *', 'yellow') )
        #print colored('Entered A star path planner with horizontal step-size %.02f [m] and horizontal step-size %.02f [m]' % (step_size_horz, step_size_vert), TERM_COLOR_INFO)
        # Set start time of start point to 0 (relative)
        point_start['time_rel'] = 0
        # Get start altitude and calc and set relative goal altitude
        point_start_alt_abs = self.get_altitude_UTM(point_start)
        point_goal_alt_abs  = self.get_altitude_UTM(point_goal)
        point_goal['z_rel'] = point_goal_alt_abs-point_start_alt_abs
        # Convert points to tuples
        point_start_tuple = self.coord_conv.pos4dDICT2pos4dTUPLE_UTM(point_start)
        point_goal_tuple  = self.coord_conv.pos4dDICT2pos4dTUPLE_UTM(point_goal)

        if type_of_planning == self.GLOBAL_PLANNING:
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_horz_step_size(step_size_horz) )
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_vert_step_size(step_size_vert) )


        neighbors_scaled = []
        for y in range(-1,+2):
            for x in range(-1,+2):
                if self.PATH_PLANNER_SPACE_DIMENSIONS == 2:
                    if not ( y == 0 and x == 0 ):
                        neighbors_scaled.append( [y*step_size_horz, x*step_size_horz, 0*step_size_vert] )
                else:
                    for z in range(-1,+2):
                        if not ( y == 0 and x == 0 and z == 0):
                            neighbors_scaled.append( [y*step_size_horz, x*step_size_horz, z*step_size_vert] )

        # Check if the path planner should continue instead of starting from scratch
        continue_planning = False
        for i in range(len(self.prev_Astar_step_size_and_points)):
            if self.prev_Astar_step_size_and_points[i] == [step_size_horz, step_size_vert, point_start, point_goal, type_of_planning] and not force_replanning:
                if self.debug:
                    print colored('Continuing search using prevoiusly computed data', TERM_COLOR_INFO)
                # Get old data
                closed_list = self.prev_Astar_closed_lists[i]
                came_from = self.prev_Astar_came_froms[i]
                g_score = self.prev_Astar_g_scores[i]
                f_score = self.prev_Astar_f_scores[i]
                open_list = self.prev_Astar_open_lists[i]
                smallest_heuristic = self.prev_Astar_smallest_heuristics[i]
                # Delete the old data
                del self.prev_Astar_step_size_and_points[i]
                del self.prev_Astar_closed_lists[i]
                del self.prev_Astar_came_froms[i]
                del self.prev_Astar_g_scores[i]
                del self.prev_Astar_f_scores[i]
                del self.prev_Astar_open_lists[i]
                del self.prev_Astar_smallest_heuristics[i]
                continue_planning = True
                if self.debug:
                    print colored(INFO_ALT_INDENT+'Continue point heuristic: %f' % smallest_heuristic, TERM_COLOR_INFO_ALT)

        if not continue_planning:
            if self.debug:
                print colored('Starting new path planning', TERM_COLOR_INFO)
            # closed list: The set of nodes already evaluated
            closed_list = set()

            # For each node, which node it can most efficiently be reached from.
            # If a node can be reached from many nodes, cameFrom will eventually contain the
            # most efficient previous step.
            came_from = {}

            # For each node, the cost of getting from the start node to that node.
            g_score = {}
            # The cost of going from start to start is zero.
            g_score[point_start_tuple] = 0

            # For each node, the total cost of getting from the start node to the goal
            # by passing by that node. That value is partly known, partly heuristic.
            f_score = {}
            # For the first node, that value is completely heuristic.
            f_score[point_start_tuple] = self.heuristic_a_star(point_start_tuple, point_goal_tuple, type_of_planning)
            if self.debug:
                print colored(INFO_ALT_INDENT+'Start point heuristic: %f' % f_score[point_start_tuple], TERM_COLOR_INFO_ALT)

            # open list: The set of currently discovered nodes that are not evaluated yet.
            open_list = []

            # Initially, only the start node is known.
            heappush(open_list, (f_score[point_start_tuple], point_start_tuple))

            # Take note of the smallest heuristic
            smallest_heuristic = INF

        # Variables for seeing the progress of the search
        open_list_popped_ctr = 0
        time_start = get_cur_time_epoch()
        time_cur = time_start
        time_last = time_start

        self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('searching', 'orange') )
        # Start searching
        while open_list and open_list_popped_ctr < max_node_exploration and time_cur-time_start < search_time_max: # Continue searching until the open_list is empty = all nodes discovered and evaluated; or max tries exceeded
            #time.sleep(10)
            if open_list_popped_ctr % 1000 == 0 and not open_list_popped_ctr == 0:
                print colored(INFO_ALT_INDENT+'Visited %i nodes from the open list in %.02f [s]' % (open_list_popped_ctr, time_cur-time_last), TERM_COLOR_INFO_ALT)
                time_last = time_cur

            current = heappop(open_list)[1] # Pop (remove and get) element with lowest f score
            open_list_popped_ctr += 1 # Increment open list pop counter
            if self.PP_PRINT_POPPED_NODE:
                tmp_f_score = f_score[current]
                tmp_g_score = g_score.get(current)
                print colored('\n'+INFO_ALT_INDENT+'Working on node (y: %.02f [m], x: %.02f [m], z_rel: %.02f [m], %.02f [s]) with F score %.02f, G score %.02f, and heuristic %.02f' % (current[0], current[1], current[2], current[3], tmp_f_score, tmp_g_score, tmp_f_score-tmp_g_score), TERM_COLOR_INFO)

            # Test if current point is close enough to the goal point
            is_goal, dist_to_goal = self.is_goal_UTM(current, point_goal_tuple, step_size_horz)
            if is_goal:
                time_cur = get_cur_time_epoch()
                self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('backtracing', 'green') )
                print colored('Path found in %.02f [s]' % (time_cur-time_start), TERM_COLOR_RES)
                planned_path = self.backtrace_path(came_from, current, point_start_tuple, point_goal_tuple)
                print colored('Path has %i waypoints\n' % (len(planned_path)), TERM_COLOR_RES)

                # for element in planned_path:
                #     print element

                planned_path = self.reduce_path_rdp_UTM(planned_path, 6*step_size_vert)

                # DRAW
                self.map_plotter.draw_path_UTM(planned_path)
                if self.DRAW_OPEN_LIST:
                    self.map_plotter.draw_points_UTM(open_list, 0, 'yellow', 2)
                if self.DRAW_CLOSED_LIST:
                    self.map_plotter.draw_points_UTM(closed_list, 1, 'grey', 5)

                self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('idle') )
                return planned_path # return the path

            # Add current node to the evaluated nodes / closed list
            closed_list.add(current)


            for y, x, z in neighbors_scaled: # Explore the neighbors
                neighbor = self.pos4dTUPLE_UTM_copy_and_move_point(current, y, x, z, type_of_planning) # Construct the neighbor node; the function calculates the 4 dimension by the time required to travel between the nodes

                # Calculate tentative g score by using the current g_score and the distance between the current and neighbor node
                tentative_g_score = g_score[current] + self.node_cost(current, neighbor, point_start_tuple, point_goal_tuple, step_size_horz, type_of_planning = type_of_planning, point_start_alt_abs = point_start_alt_abs)

                if self.PP_PRINT_EXPLORE_NODE:
                    print colored(INFO_ALT_INDENT+'Exploring node (%.02f [m], %.02f [m], %.02f [m], %.02f [s]) with tentative G score %.02f, prevoius G score %.02f (0 = node not visited)' % (neighbor[0], neighbor[1], neighbor[2], neighbor[3], tentative_g_score, g_score.get(neighbor, 0)), TERM_COLOR_INFO_ALT)

                # Maybe test here if inside no-fly zones or violate aircrafts and skip if needed NOTE TODO see a-star-test.py
                if self.is_point_in_no_fly_zones_UTM(neighbor, step_size_horz)[0]:
                    #print 'INSIDE GEOFENCE'
                    continue

                if neighbor[2]+step_size_vert <= self.get_altitude_UTM(neighbor)-point_start_alt_abs: # note that <= is to include points on the surface (it is a drone, not a car)
                    #print 'Below the earths surface, neighbor rel altitude %.02f, neighbor abs altitude %.02f, start abs altitude %.02f' % (neighbor[2], self.get_altitude_UTM(neighbor), point_start_alt_abs)
                    continue

                if neighbor[2] > self.MAX_FLYING_ALTITUDE:
                    print 'Violates the %.01f altitude legislated limit'
                    continue

                # Ignore the neighbor which is already evaluated and test if a better path is found to the neighbor node (tentative_g_score is smaller than the g_score stored in the g_score dict (the 0 is the default value if the element does not exist) = better path)
                if neighbor in closed_list and tentative_g_score >= g_score.get(neighbor, 0):
                    #print colored('Node already visited', 'yellow')
                    continue

                # Test if the tentative_g_score is smaller than the one stored in g_score (if the node is new it is not stored in the g_score list) or if the neighbor is not part of the open list ([i[1]for i in open_list] simply returns the point values)
                if  tentative_g_score < g_score.get(neighbor, 0) or neighbor not in [ i[1] for i in open_list ]:
                    #print colored('Choosing node and adding to list', 'green')
                    came_from[neighbor] = current # Add the parent to the came from list
                    g_score[neighbor] = tentative_g_score # Add the tentative g score to the g_score list
                    #print colored('Heuristic: '+str(self.heuristic_a_star(neighbor, point_goal_tuple)),'yellow')
                    neighbor_heiristic = self.heuristic_a_star(neighbor, point_goal_tuple, type_of_planning)
                    if neighbor_heiristic < smallest_heuristic:
                        smallest_heuristic = neighbor_heiristic
                        self.gui.on_main_thread( lambda: self.gui.set_global_plan_cur_heuristic(smallest_heuristic) )
                    f_score[neighbor] = tentative_g_score + neighbor_heiristic#self.heuristic_a_star(neighbor, point_goal_tuple) # Calculate and add the f score (combination of g score and heuristic)
                    heappush(open_list, (f_score[neighbor], neighbor)) # Add the node to the open list
            time_cur = get_cur_time_epoch()
            if (round(time_cur,1) - round(time_start,1)) % 0.5 == 0:
                self.gui.on_main_thread( lambda: self.gui.set_global_plan_search_time( time_cur - time_start ) )
                self.gui.on_main_thread( lambda: self.gui.set_global_plan_nodes_visited( len(closed_list) ) )
                self.gui.on_main_thread( lambda: self.gui.set_global_plan_nodes_explored( len(open_list) ) )

        # No solution found
        if time_cur-time_start >= search_time_max:
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('time limit reached', 'red') )
        elif open_list_popped_ctr >= max_node_exploration:
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('node limit reached', 'red') )
        else:
            self.gui.on_main_thread( lambda: self.gui.set_global_plan_status('no solution exists', 'red') )
        print colored('A star path planner stopped, visited %i nodes (max %i) in %.02f [s] (max %.02f [s]), open list size %i, closed list size %i\n' % (open_list_popped_ctr, max_node_exploration, time_cur-time_start, search_time_max, len(open_list), len(closed_list)), TERM_COLOR_SUB_RES)

        # Save the planning data so it can be resumed
        self.prev_Astar_step_size_and_points.append( [step_size_horz, step_size_vert, point_start, point_goal, type_of_planning] )
        self.prev_Astar_closed_lists.append( closed_list )
        self.prev_Astar_open_lists.append( open_list )
        self.prev_Astar_came_froms.append( came_from )
        self.prev_Astar_g_scores.append( g_score )
        self.prev_Astar_f_scores.append( f_score )
        self.prev_Astar_smallest_heuristics.append( smallest_heuristic )

        # Draw start and goal points so plot is not empty
        self.map_plotter.draw_circle_UTM(point_start_tuple, 'green')
        self.map_plotter.draw_circle_UTM(point_goal_tuple, 'green')
        return [] # No path found

    def heuristic_a_star(self, point4dUTM1, point4dUTM2, type_of_planning = 0):
        """
        Calculates the heuristic used by the A star algorithm to know how close the current node is to the goal node
        Input: 2 4d UTM points; presumably current node and goal node
        Output: scalar heuristic; current unit [m]
        """
        if type_of_planning == self.EMERGENCY_PLANNING:
            # Use the closest rally point for the heuristic, TODO
            pass
        heuristic = self.calc_euclidian_dist_UTM(point4dUTM1, point4dUTM2)[0] # element 0 = total3D_dist, element 1 = horz distance, element 2 = vert distance
        #heuristic = self.calc_euclidian_horz_dist_UTM(point4dUTM1, point4dUTM2)
        return heuristic

    def node_cost(self, parent_point4dUTM, child_point4dUTM, start_point4dUTM, goal_point4dUTM, step_size_horz, type_of_planning = 0, point_start_alt_abs = 0):
        """
        Calculates the child node cost based on the parent node
        Input: 4 4d UTM points; 1 parent, 1 child node, 1 start node, and 1 goal node.
                optional type of planning
                    0: global planning (no-fly zones, height map)
                    1: local planning (no-fly zones, aircrafts, gliders, drones)
                    2: emergency planning (no-fly zones, aircrafts, gliders, drones, rally points)
        Output: scalar node cost [unitless]
        """
        node_cost = 0
        if type_of_planning == self.GLOBAL_PLANNING:
            # Calculate the travel time from the already present info (computed in copy_and_move)
            #travel_time = child_point4dUTM[3]-parent_point4dUTM[3] # The global planner does not use time!
            travel_time = self.calc_travel_time_from_UTMpoints(child_point4dUTM, parent_point4dUTM)
            total_dist_seg, seg_horz_distance_seg, seg_vert_distance_seg = self.calc_euclidian_dist_UTM(parent_point4dUTM, child_point4dUTM)

            # Calculate the distance to the start and goal node to see if the altutide whould be affected
            #total_dist_start, horz_distance_start, vert_distance_start = self.calc_euclidian_dist_UTM(start_point4dUTM, child_point4dUTM)
            total_dist_goal, horz_distance_goal, vert_distance_goal = self.calc_euclidian_dist_UTM(goal_point4dUTM, child_point4dUTM)

            # Calculate the height of the child point
            point_child_alt_abs = self.get_altitude_UTM(child_point4dUTM)
            point_child_et_start_alt_diff = point_child_alt_abs - point_start_alt_abs
            #print point_start_alt_abs, point_child_alt_abs, point_child_et_start_alt_diff, self.IDEAL_FLIGHT_ALTITUDE+point_child_et_start_alt_diff, child_point4dUTM[2], abs(self.IDEAL_FLIGHT_ALTITUDE+point_child_et_start_alt_diff - child_point4dUTM[2])

            # Penalize for flight altitude
            if horz_distance_goal > step_size_horz: # Not close to the goal - keep ideal altitude
                node_cost += abs(self.IDEAL_FLIGHT_ALTITUDE+point_child_et_start_alt_diff - child_point4dUTM[2])
            else: # Close to the goal - move towards goal altitude
                node_cost += abs(goal_point4dUTM[2]-child_point4dUTM[2])

            # Penalize for travel time
            node_cost += travel_time #0.1*total_dist + 0.1*travel_time

            # Penalize for flight distance
            node_cost += total_dist_seg

            return node_cost

        elif type_of_planning == self.LOCAL_PLANNING:

            return 0

        elif type_of_planning == self.EMERGENCY_PLANNING:
            # Move towards rally points - linear function to closest
            return 0

        else:
            if self.debug:
                print colored('Type of planning; node cost defaulted to 0', TERM_COLOR_ERROR)
            return 0


    def print_a_star_open_list_nice(self, list_in):
        print colored('\nPrinting the open list; contains %i elements' % len(list_in), TERM_COLOR_INFO)
        for i in range(len(list_in)):
            print colored(INFO_ALT_INDENT+'Element %i: heuristic %f, y %.02f [m], x %.02f [m], z_rel %.02f [m], time_rel %.02f ' % (i, list_in[i][0], list_in[i][1][0], list_in[i][1][1], list_in[i][1][2], list_in[i][1][3]), TERM_COLOR_INFO_ALT)
    def print_a_star_closed_list_nice(self, list_in):
        print colored('\nPrinting the closed list (set); contains %i elements' % len(list_in), TERM_COLOR_INFO)
        itr = 0
        for element in list_in:
            print colored(INFO_ALT_INDENT+'Element %i: y %.02f [m], x %.02f [m], z_rel %.02f [m], time_rel %.02f ' % (itr, element[0], element[1], element[2], element[3]), TERM_COLOR_INFO_ALT)
            itr += 1

    def backtrace_path(self, came_from, current_node, start_node, goal_node):
        """
        Backtraces from the goal node (or node near goal node) given by current node to the start node using the came_from data
        Input: came_from list, current_node to backtrace from, and start node to trace to
        Output: backtraced node as a array of UTM points4d
        """
        path = [] # make empty array to hold the path
        while current_node in came_from:
            path.append(self.coord_conv.pos4dTUPLE2pos4dDICT_UTM(current_node)) # add the current node to the path
            current_node = came_from[current_node] # get the parent node
        path.append(self.coord_conv.pos4dTUPLE2pos4dDICT_UTM(start_node))
        path.reverse()
        # Since the last point is within the acceptance radius of the goal point, the x, y, and z_rel of the last point is replaced with the data from the goal point and the added travel time is calculated and added
        tmp_second_last_point_arr = [path[-2]['y'], path[-2]['x'], path[-2]['z_rel']]
        path[-1]['time_rel'] = path[-2]['time_rel'] + self.calc_travel_time_from_UTMpoints(tmp_second_last_point_arr, goal_node)
        path[-1]['y'] = goal_node[0] # element 0 = y
        path[-1]['x'] = goal_node[1] # element 1 = x
        path[-1]['z_rel'] = goal_node[2] # element 2 = z_rel
        return path

    def reduce_path_rdp_UTM(self, planned_path, tolerance=-1):
        """
        Removes waypoints according to the Ramer-Douglas-Peucker algorithm
        Input: UTM planned path (a set of 4d UTM tuple points) and optional tolerance [m]
        Output: optimized UTM planned path
        """
        hemisphere = planned_path[0]['hemisphere']
        zone = planned_path[0]['zone']
        letter = planned_path[0]['letter']

        planned_path_yxz = []
        for element in planned_path:
            planned_path_yxz.append([element['y'], element['x'], element['z_rel']])

        if tolerance == -1:
            planned_path_yxz = rdp(planned_path_yxz)
            # NOTE this is the same as the algorithm 'reduce_path_simple_straight_line_UTM'
        else:
            planned_path_yxz = rdp(planned_path_yxz, tolerance)

        planned_path = []
        # make the right format again
        for i in range(len(planned_path_yxz)):
            if i == 0:
                planned_path.append({'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':planned_path_yxz[i][0],'x':planned_path_yxz[i][1],'z_rel':planned_path_yxz[i][2],'time_rel':0})
            else:
                travel_time_abs = planned_path[i-1]['time_rel'] + self.calc_travel_time_from_UTMpoints(planned_path_yxz[i-1], planned_path_yxz[i])
                planned_path.append({'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':planned_path_yxz[i][0],'x':planned_path_yxz[i][1],'z_rel':planned_path_yxz[i][2],'time_rel':travel_time_abs})

        return planned_path

    def pos4dTUPLE_UTM_copy_and_move_point(self, point4dUTM, y_offset, x_offset, z_offset, type_of_planning = 0):
        """
        Takes a pos4d tuple and modifies it to a new pos4d tuple using the provided offsets
        Input: 1 4d UTM point along with 3 dimensional offset given as individual arguments
        Output: pos4dTUPLE
        """
        tmp_point_arr   = [point4dUTM[0]+y_offset, point4dUTM[1]+x_offset, point4dUTM[2]+z_offset, point4dUTM[3], point4dUTM[4], point4dUTM[5], point4dUTM[6]] # make tmp arr because it is a mutable container
        tmp_point_tuple = (tmp_point_arr[0], tmp_point_arr[1], tmp_point_arr[2], tmp_point_arr[3], tmp_point_arr[4], tmp_point_arr[5], tmp_point_arr[6]) # make tmp tuple for calculating the travel time
        if type_of_planning == self.GLOBAL_PLANNING:
            travel_time_delta = 0
        else:
            travel_time_delta = self.calc_travel_time_from_UTMpoints(point4dUTM, tmp_point_tuple)
        tmp_point_arr[3] = tmp_point_arr[3] + travel_time_delta
        return (tmp_point_arr[0], tmp_point_arr[1], tmp_point_arr[2], tmp_point_arr[3], tmp_point_arr[4], tmp_point_arr[5], tmp_point_arr[6])

    def calc_euclidian_dist_UTM(self, point4dUTM1, point4dUTM2):
        """
        Calculates the euclidian distance (3d) between 2 UTM points
        Input: 2 UTM points
        Output: euclidian distance, euclidian horizontal distance (2d), and vertical distance (1d)
        """
        if isinstance(point4dUTM1, dict) and isinstance(point4dUTM2, dict):
            total3d = sqrt( pow(point4dUTM1['x']-point4dUTM2['x'],2) + pow(point4dUTM1['y']-point4dUTM2['y'],2) + pow(point4dUTM1['z_rel']-point4dUTM2['z_rel'],2) )
            vert_distance = abs(point4dUTM1['z_rel']-point4dUTM2['z_rel'])
        elif ( isinstance(point4dUTM1, tuple) or isinstance(point4dUTM1, list) ) and ( isinstance(point4dUTM2, tuple) or isinstance(point4dUTM2, list) ):
            total3d = sqrt( pow(point4dUTM1[1]-point4dUTM2[1],2) + pow(point4dUTM1[0]-point4dUTM2[0],2) + pow(point4dUTM1[2]-point4dUTM2[2],2) )
            vert_distance = abs(point4dUTM1[2]-point4dUTM2[2])
        else:
            print colored('Cannot calculate euclidian distance because formats do not match', TERM_COLOR_ERROR)
            total3d = INF
            vert_distance = INF
        horz_distance = self.calc_euclidian_horz_dist_UTM(point4dUTM1, point4dUTM2)
        return total3d, horz_distance, vert_distance

    def calc_euclidian_horz_dist_UTM(self, point4dUTM1, point4dUTM2):
        """
        Calculates the horizontal euclidian distance (2d) between 2 UTM points
        Input: 2 UTM points
        Output: horizontal euclidian distance
        """
        if isinstance(point4dUTM1, dict) and isinstance(point4dUTM2, dict):
            return sqrt( pow(point4dUTM1['x']-point4dUTM2['x'],2) + pow(point4dUTM1['y']-point4dUTM2['y'],2) )
        elif ( isinstance(point4dUTM1, tuple) or isinstance(point4dUTM1, list) ) and ( isinstance(point4dUTM2, tuple) or isinstance(point4dUTM2, list) ):
            return sqrt( pow(point4dUTM1[1]-point4dUTM2[1],2) + pow(point4dUTM1[0]-point4dUTM2[0],2) )
        else:
            print colored('Cannot calculate euclidian horizontal distance because formats do not match', TERM_COLOR_ERROR)
            return INF

    def calc_travel_time_from_UTMpoints(self, point4dUTM1, point4dUTM2):
        """
        Calculates the travel time by combining the horizontal and vertical travel times using pythagoras
        Input: 2 4d UTM points
        Output: travel time [s]
        """
        dist, horz_dist, vert_dist = self.calc_euclidian_dist_UTM(point4dUTM1, point4dUTM2)
        return ( (horz_dist / self.UAV_NOMINAL_AIRSPEED_HORZ_MPS) + ( vert_dist / self.UAV_NOMINAL_AIRSPEED_VERT_MPS) )
    def calc_travel_time_from_dist(self, horz_dist, vert_dist):
        """
        Calculates the travel time by combining the horizontal and vertical travel times using pythagoras
        Input: horizontal and vertical distances
        Output: travel time [s]
        """
        return ( (horz_dist / self.UAV_NOMINAL_AIRSPEED_HORZ_MPS) + ( vert_dist / self.UAV_NOMINAL_AIRSPEED_VERT_MPS) )
    def is_goal_UTM(self, point4dUTM_test, point4dUTM_goal, in_goal_acceptance_radius_m):
        """
        Checks if the distance between the test point and the gaol points are within the allowed acceptance radius
        Input: 2 4d UTM points
        Output: True if the test point is within the acceptance radius and False if not along with the distance to the goal
        """
        # dist = self.calc_euclidian_horz_dist_UTM(point4dUTM_test, point4dUTM_goal)
        dist, horz_dist, vert_dist = self.calc_euclidian_dist_UTM(point4dUTM_test, point4dUTM_goal)
        if dist > in_goal_acceptance_radius_m:
            # if self.debug:
            #     print colored(error_indent+'Points are NOT within goal acceptance radius; distance between points: %.02f [m], acceptance radius: %.02f [m]' % (dist, self.goal_acceptance_radius), TERM_COLOR_INFO_ALT)
            return False, dist
        else:
            if self.debug:
                print colored(RES_INDENT+'Node is within goal acceptance radius; distance between node and goal: %.02f [m], acceptance radius: %.02f [m]' % (dist, in_goal_acceptance_radius_m), TERM_COLOR_RES)
            return True, dist



    """ Path planner evaluator functions """
    def evaluate_path(self):
        """
        Calculates the fitness score of a path by evaluating the different path elements
        Input: none but uses self.planned_path_global_geodetic
        Output: none
        """
        # Check if the path is a path (more than 2 waypoints)
        if len(self.planned_path_global_geodetic) >= 2:
            print colored('\nEvaluating path', TERM_COLOR_INFO)
            # Convert to / ensure pos4dDICT object
            path_converted = []
            try:
                tmp_var = self.planned_path_global_geodetic[0]['lat']
            except TypeError:
                print 'converting'
                for i in range(len(self.planned_path_global_geodetic)):
                    path_converted.append( self.coord_conv.pos4d2pos4dDICT_geodetic( self.planned_path_global_geodetic[i] ) )
            else:
                path_converted = self.planned_path_global_geodetic

            horz_distance, horz_distances = self.calc_horz_dist_geodetic_total(path_converted)
            vert_distance, vert_distance_ascend, vert_distance_descend, vert_distances = self.calc_vert_dist(path_converted)
            travel_time = self.calc_travel_time(path_converted)
            if self.debug:
                print colored('Calculating number of waypoints', TERM_COLOR_INFO)
            waypoints = len(path_converted)
            if self.debug:
                print colored(SUB_RES_INDENT+'Total waypoints %d' % waypoints, TERM_COLOR_SUB_RES)
            avg_waypoint_dist, total3d_waypoint_dist = self.calc_avg_waypoint_dist(path_converted, horz_distances, vert_distances)

            fitness = self.HORZ_DISTANCE_FACTOR*horz_distance
            fitness = fitness + self.VERT_DISTANCE_FACTOR*vert_distance
            fitness = fitness + self.TRAVEL_TIME_FACTOR*travel_time
            fitness = fitness + self.WAYPOINTS_FACTOR*waypoints
            fitness = fitness + self.AVG_WAYPOINT_DIST_FACTOR*avg_waypoint_dist
            print colored('Path evaluation done', TERM_COLOR_INFO)
            print colored(RES_INDENT+'Path fitness %.02f [unitless]' % fitness, TERM_COLOR_RES)
            self.planned_path_global_fitness = fitness

            self.memory_usage_module.save_heap() # Save the heap size before calculating and outputting statistics

            runtime_pp_s = self.time_planning_completed_s - self.time_planning_start_s # Calculate runtime

            # Calculate statistics
            used_path_planner = self.PATH_PLANNER_NAMES[self.PATH_PLANNER]
            estimated_flight_time = self.planned_path_global_geodetic[len(self.planned_path_global_geodetic)-1]['time'] - self.planned_path_global_geodetic[0]['time']

            byte_amount = self.memory_usage_module.get_bytes()
            byte_amount_diff = self.memory_usage_module.get_bytes_diff()
            object_amount = self.memory_usage_module.get_objects()
            object_amount_diff = self.memory_usage_module.get_objects_diff()

            no_waypoints = len(self.planned_path_global_geodetic)

            # Print statistics
            if self.PRINT_STATISTICS:
                print colored('\n                              Path planner: %s' % used_path_planner, 'yellow')
                print colored('                              Path fitness: %f' % fitness, 'yellow')
                print colored('                            Total distance: %.02f [m]' % total3d_waypoint_dist, 'yellow')
                print colored('                       Horizontal distance: %.02f [m]' % horz_distance, 'yellow')
                print colored('                         Vertical distance: %.02f [m]' % vert_distance, 'yellow')
                print colored('                      Runtime path planner: %f [s] (%s)' % (runtime_pp_s, str(convert_time_delta2human(runtime_pp_s))), 'yellow')
                print colored('                      Number of bytes used: %i' % byte_amount, 'yellow')
                print colored('  Number of bytes used by the path planner: %i' % byte_amount_diff, 'yellow')
                print colored('                    Number of objects used: %i' % object_amount, 'yellow')
                print colored('Number of objects used by the path planner: %i' % object_amount_diff, 'yellow')
                print colored('                     Estimated flight time: %.02f [s] (%s)' % (estimated_flight_time, str(convert_time_delta2human(estimated_flight_time))), 'yellow') # TODO
                print colored('                       Number of waypoints: %i' % (no_waypoints), 'yellow')
                print colored('                                      Path: %s' % (str(self.planned_path_global_geodetic)), 'yellow')

            # Save statistics to file
            if self.SAVE_STATISTICS_TO_FILE:
                now = get_cur_time_datetime()
                file_name = ('PP_%d-%02d-%02d-%02d-%02d.csv' % (now.year, now.month, now.day, now.hour, now.minute))
                sub_folder = 'results'
                file_name = sub_folder+'/'+file_name
                output_file_CSV = open(file_name, 'w')
                output_writer_CSV = csv.writer(output_file_CSV,quoting=csv.QUOTE_MINIMAL)
                fields = ['path planner', 'path fitness [unitless]', 'total distance [m]', 'horizontal distance [m]', 'vertical distance [m]', 'runtime path planner [s]', 'bytes used', 'bytes used by the path planner', 'objects used', 'objects used by the path planner', 'flight time [s]', 'waypoints', 'start point', 'goal point', 'path']
                output_writer_CSV.writerow(fields)
                data = [used_path_planner, fitness, total3d_waypoint_dist, horz_distance, vert_distance, runtime_pp_s, byte_amount, byte_amount_diff, object_amount, object_amount_diff, estimated_flight_time, no_waypoints, self.point_start_global_geodetic, self.point_goal_global_geodetic, self.planned_path_global_geodetic]
                output_writer_CSV.writerow(data)
                output_file_CSV.close()
        else:
            print colored('\nCannot evaluate path because it has fewer than 2 waypoints', TERM_COLOR_ERROR)

    def calc_horz_dist_geodetic_total(self, path):
        """
        Calculates the total horizontal distance from a geodetic path
        Input: path of geodetic corrdinates
        Output: total horizontal distance [m] along with a list of the horizontal distance for the individual path elements [m]
        """
        if self.debug:
            print colored('Calculating horizontal path distance (2D)', TERM_COLOR_INFO)
        horz_distances = []
        total_horz_distance = 0 # unit: m
        for i in range(len(path)-1):
            tmp_horz_distance = self.calc_horz_dist_geodetic(path[i]['lat'], path[i]['lon'], path[i+1]['lat'], path[i+1]['lon'])
            total_horz_distance = total_horz_distance + tmp_horz_distance
            horz_distances.append(tmp_horz_distance)
        if self.debug:
            print colored(SUB_RES_INDENT+'Total horizontal distance %.01f [m]' % total_horz_distance, TERM_COLOR_SUB_RES)
        return total_horz_distance, horz_distances
    def calc_horz_dist_geodetic(self, lat1, lon1, lat2, lon2):
        """
        Calculates the great circle distance (only horizontal, 2d) distance between 2 pairs of geodetic latitude and longitude
        Input: 2 pairs of geodetic latitude and longitude; representing 2 points
        Output: Great circle distance between points
        """
        # https://jswhit.github.io/pyproj/pyproj.Geod-class.html
        az12, az21, dist = self.GEOID_WGS84.inv(lon1,lat1,lon2,lat2)
        return dist

    def calc_vert_dist(self, path):
        """
        Calculates the vertical distance from a geodetic path
        Input: path of geodetic corrdinates
        Output: total vertical distance (sum of abs vertical distances), vertical ascend distance, vertical descend distance, and a list of the vertical distance for the individual path elements [m] (signed)
        """
        if self.debug:
            print colored('Calculating vertical path distance (1D)', TERM_COLOR_INFO)
        vert_distances = []
        total_vert_distance = 0 # unit: m
        total_vert_distance_ascend = 0 # unit: m
        total_vert_distance_descend = 0 # unit: m
        for i in range(len(path)-1):
            tmp_vert_distance = path[i+1]['alt_rel']-path[i]['alt_rel']
            if tmp_vert_distance < 0:
                total_vert_distance_descend = total_vert_distance_descend + abs(tmp_vert_distance)
            else:
                total_vert_distance_ascend = total_vert_distance_ascend + tmp_vert_distance
            total_vert_distance = total_vert_distance + abs(tmp_vert_distance)
            vert_distances.append(tmp_vert_distance)
        if self.debug:
            print colored(SUB_RES_INDENT+'Total vertical distance %.01f [m], ascend distance %.01f [m], descend distance %.01f [m]' % (total_vert_distance, total_vert_distance_ascend, total_vert_distance_descend), TERM_COLOR_SUB_RES)
        return total_vert_distance, total_vert_distance_ascend, total_vert_distance_descend, vert_distances

    def calc_travel_time(self, path):
        """
        Calculates the travel time the timestamps in a geodetic path
        Input: path of geodetic corrdinates with timestamp (handles both relative and absoulte time)
        Output: travel time [s]
        """
        if self.debug:
            print colored('Calculating travel time', TERM_COLOR_INFO)
        travel_time = 0 # unit: s
        try:
            tmp_var = path[0]['time_rel']
        except KeyError:
            for i in range(len(path)-1):
                travel_time = travel_time + (path[i+1]['time'] - path[i]['time'])
        else:
            for i in range(len(path)-1):
                travel_time = travel_time + (path[i+1]['time_rel'] - path[i]['time_rel'])

        if self.debug:
            print colored(SUB_RES_INDENT+'Travel time %.0f [s] (%s)' % (travel_time, str(convert_time_delta2human(travel_time))), TERM_COLOR_SUB_RES)
        return travel_time

    def calc_avg_waypoint_dist(self, path, horz_distances=False, vert_distances=False):
        """
        Calculates the average distance between waypoints in a geodetic path
        Input: path of geodetic corrdinates along with optional list of horizontal and vertical distances (optimization not to calculate the great circle distances more than once)
        Output: the average waypoint distance and the total 3d distance of the path
        """
        if self.debug:
            print colored('Calculating average waypoint distance', TERM_COLOR_INFO)
        # Check if the required distances have already been calculated, otherwise calculate
        if ((horz_distances != False and isinstance(horz_distances, list)) and (vert_distances != False and isinstance(vert_distances, list))) == False:
            horz_distances = self.calc_horz_dist_geodetic_total(path)[1]
            vert_distances = self.calc_vert_dist(path)[3]
        avg_waypoint_dist = 0 # unit: m
        total3d_waypoint_dist = 0 # unit: m
        if len(horz_distances) == len(vert_distances):
            for i in range(len(horz_distances)):
                # Calculate combined distance from horizontal and vertical using pythagoras
                total3d_waypoint_dist = total3d_waypoint_dist + sqrt( pow(horz_distances[i], 2) + pow(vert_distances[i], 2) )
            avg_waypoint_dist = total3d_waypoint_dist / len(horz_distances)
        else:
            print colored('Input does not match in size, average waypoint distance set to 0 [m]', TERM_COLOR_ERROR)
        if self.debug:
            print colored(SUB_RES_INDENT+'Average waypoint distance %f [m]' % avg_waypoint_dist, TERM_COLOR_SUB_RES)
        return avg_waypoint_dist, total3d_waypoint_dist

    """ Other helper functions """
    def print_path_raw(self):
        """ Prints the planned geodetic path in a raw format """
        if len(self.planned_path_global_geodetic) >= 1:
            print colored('\n'+SUB_RES_INDENT+str(self.planned_path_global_geodetic), TERM_COLOR_SUB_RES)
        else:
            print colored('\nCannot print path because it is empty', TERM_COLOR_ERROR)
    def print_path_nice(self):
        """ Prints the input self.planned_path_global_geodetic in a human readable format """
        if len(self.planned_path_global_geodetic) >= 1:
            print colored('\nPlanned self.planned_path_global_geodetic:', TERM_COLOR_RES)
            try:
                tmp_var = self.planned_path_global_geodetic[0]['time']
            except KeyError:
                try:
                    tmp_var = self.planned_path_global_geodetic[0]['alt']
                except KeyError:
                    for i in range(len(self.planned_path_global_geodetic)):
                        print colored(SUB_RES_INDENT+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, self.planned_path_global_geodetic[i]['lat'], self.planned_path_global_geodetic[i]['lon'], self.planned_path_global_geodetic[i]['alt_rel'], self.planned_path_global_geodetic[i]['time_rel']), TERM_COLOR_SUB_RES)
                else:
                    for i in range(len(self.planned_path_global_geodetic)):
                        print colored(SUB_RES_INDENT+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, self.planned_path_global_geodetic[i]['lat'], self.planned_path_global_geodetic[i]['lon'], self.planned_path_global_geodetic[i]['alt'], self.planned_path_global_geodetic[i]['time_rel']), TERM_COLOR_SUB_RES)
            else:
                try:
                    tmp_var = self.planned_path_global_geodetic[0]['alt']
                except KeyError:
                    for i in range(len(self.planned_path_global_geodetic)):
                        print colored(SUB_RES_INDENT+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, self.planned_path_global_geodetic[i]['lat'], self.planned_path_global_geodetic[i]['lon'], self.planned_path_global_geodetic[i]['alt_rel'], self.planned_path_global_geodetic[i]['time']), TERM_COLOR_SUB_RES)
                else:
                    for i in range(len(self.planned_path_global_geodetic)):
                        print colored(SUB_RES_INDENT+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, self.planned_path_global_geodetic[i]['lat'], self.planned_path_global_geodetic[i]['lon'], self.planned_path_global_geodetic[i]['alt'], self.planned_path_global_geodetic[i]['time']), TERM_COLOR_SUB_RES)
        else:
            print colored('\nCannot print the geodetic planed path because it is empty', TERM_COLOR_ERROR)
    def convert_rel2abs_time(self, start_time_epoch = False):
        """
        Converts the relative time to absoulte time by adding the current time or provided time of the start point to each point in the path
        Input: path and optional start time (all relative times are relative to the time at the starting point)
        Output: none but changes the input path and changes the key from 'time_rel' to 'time'
        """
        if len(self.planned_path_global_geodetic) > 0:
            if start_time_epoch == False:
                start_time_epoch = get_cur_time_epoch_wo_us()
            for i in range(len(self.planned_path_global_geodetic)):
                self.planned_path_global_geodetic[i]['time'] =self.planned_path_global_geodetic[i].pop('time_rel')
                self.planned_path_global_geodetic[i]['time'] = self.planned_path_global_geodetic[i]['time'] + start_time_epoch
    def convert_rel2abs_alt(self, abs_heigh_start_point):
        """
        Converts the relative heights to absoulte heights by adding the absoulute height of the start point to each point in the path
        Input: path and absoulute height of start point (all relative heights are relative to the height at the starting point)
        Output: none but changes the input path and changes the key from 'alt_rel' to 'alt'
        """
        if len(self.planned_path_global_geodetic) > 0:
            for i in range(len(self.planned_path_global_geodetic)):
                self.planned_path_global_geodetic[i]['alt'] =self.planned_path_global_geodetic[i].pop('alt_rel')

if __name__ == '__main__':
    # Save the start time before anything else
    time_task_start_s = get_cur_time_epoch()

    # Set up the logger for the bokeh lib
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)

    # Instantiate memory_usage class
    memory_usage_module = memory_usage()
    memory_usage_module.save_heap_start() # Save the heap size before calculating and outputting statistics

    ## Test points
    start_point_3dDICT = {'lat': 55.431122, 'lon': 10.420436, 'alt_rel': 0} # org
    start_point_3dDICT = {'lat': 55.431122, 'lon': 10.420436, 'alt_rel': 0} # more north west
    start_point_3dDICT = {'lat': 55.435259, 'lon': 10.410862, 'alt_rel': 0} # even more north west
    goal_point_3dDICT  = {'lat': 55.427750, 'lon': 10.433737, 'alt_rel': 0} # more north east
    goal_point_3dDICT  = {'lat': 55.427043, 'lon': 10.450245, 'alt_rel': 0} # even more north east
    goal_point_3dDICT  = {'lat': 55.424736, 'lon': 10.419749, 'alt_rel': 0} # more south
    #goal_point_3dDICT  = {'lat': 55.427203, 'lon': 10.419043, 'alt_rel': 0} # org

    # Instantiate UAV_path_planner class
    UAV_path_planner_module = UAV_path_planner(True, data_path_no_fly_zones = 'data_sources/no_fly_zones/drone_nofly_dk.kml', data_path_height_map = 'data_sources/height/')
    """
    No-fly zone paths: (local) 'data_sources/no_fly_zones/drone_nofly_dk.kml' = old but has airports etc.
                       (local) 'data_sources/no_fly_zones/KmlUasZones_2018-05-02-15-50.kml' = current from droneluftrum.dk
                       (local) 'data_sources/no_fly_zones/KmlUasZones_sec2.kml' = only contains 4 no-fly zones from Odense
                       Not specified = load online
    Height map path:    (local) 'data_sources/height/'
    """

    """ Path planning start """
    # Plan path
    #UAV_path_planner_module.plan_path_global(start_point_3dDICT, goal_point_3dDICT)
    # Print planned path
    #UAV_path_planner_module.print_path_nice()
    #UAV_path_planner_module.print_path_raw()
    # Convert the time from relative to absolute
    #UAV_path_planner_module.convert_rel2abs_time()
    # Evaluate path and also note the total distance, horizontal distance, and vertical distance for the statistics
    #UAV_path_planner_module.evaluate_path()

    # Show a plot of the planned path
    #UAV_path_planner_module.map_plotter.show_plot()

    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True
    # Stop the GUI thread
    UAV_path_planner_module.stop_gui()
