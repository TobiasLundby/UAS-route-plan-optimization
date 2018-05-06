#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-04-10 TL First version
"""

"""
Description:
Path planner for UAVs
License: BSD 3-Clause
"""

""" Import libraries """
from math import pow, sqrt, pi, cos
from termcolor import colored
from pyproj import Geod
import datetime
import time
import datetime # datetime.now
import pytz # timezones in the datetime format
from copy import copy, deepcopy
import csv # for saving statistics and path
from heapq import * # for the heap used in the A star algorithm
from libs.coordinate import coordinate_transform
from libs.map_plotter import map_plotter
from libs.memory_usage import memory_usage
from data_sources.no_fly_zones.kml_reader import kml_no_fly_zones_parser
from shapely import geometry # used to calculate the distance to polygons
from rdp import rdp

""" User constants """
PRINT_STATISTICS        = True
SAVE_STATISTICS_TO_FILE = False

class UAV_path_planner():
    """ UAV constants """
    uav_nominal_airspeed_horz_mps   = 15 # unit: m/s
    uav_nominal_airspeed_vert_mps   = 5  # unit: m/s
    uav_nominal_battery_time_min    = 20 # unit: min
    uav_nominal_battery_time_s      = uav_nominal_battery_time_min*60 # unit: s
    """ Path planning constants """
    PATH_PLANNER_ASTAR              = 0 # just an internal number to recognize the path planner
    PATH_PLANNER_NAMES              = ['A star algorithm']
    PATH_PLANNER                    = PATH_PLANNER_ASTAR # the name of the chosen path path planner; NOTE the user can set this
    goal_acceptance_radius          = 10 # unit: m
    map_horz_step_size              = 10 # unit: m
    # neighbors in 8 connect 2d planning; values can be scaled if needed
    neighbors                       = [ [0,1,0], [0,-1,0], [1,0,0], [-1,0,0], [1,1,0], [1,-1,0], [-1,1,0], [-1,-1,0] ]
    # neighbors in 4 connect 2d planning; values can be scaled if needed
    #neighbors = [ [0,1,0], [0,-1,0], [1,0,0], [-1,0,0] ]
    neighbors_scaled                = []
    PP_max_node_exploration         = 4294967295
    print_popped_node               = True
    print_explore_node              = False
    draw_open_list                  = False
    draw_closed_list                = False
    no_fly_zone_buffer_zone         = 0.0 # unit: m
    """ Terminal output color constants """
    default_term_color_info         = 'cyan'
    default_term_color_info_alt     = 'magenta'
    info_alt_indent                 = ' -- '
    default_term_color_error        = 'red'
    error_indent                    = ' ÷÷ '
    default_term_color_tmp_res      = 'yellow'
    tmp_res_indent                  = ' -> '
    default_term_color_res          = 'green'
    res_indent                      = ' ++ '
    """ Path evaluation (path fitness) factor constants """
    horz_distance_factor            = 1 # unit: unitless
    vert_distance_factor            = 2 # unit: unitless
    travel_time_factor              = 1 # unit: unitless
    waypoints_factor                = 1 # unit: unitless
    avg_waypoint_dist_factor        = 1 # unit: unitless
    """ Other constants """
    geofence_height                 = 100 # unit: m
    no_fly_reduce_flight_time_factor = 2 # unit: unitless ; reason: if the wind is 12m/s and the drone if flying with 15m/s the safe reducing is 1.8 (not safe) ≈ 2 (safe)
    forever                         = 60*60*24*365*100  # unit: s; 100 years excl. leap year
    inf                             = 4294967295 # 32bit from 0
    geoid_distance                  = Geod(ellps='WGS84') # used for calculating Great Circle Distance

    def __init__(self, debug = False):
        """ Constructor """
        self.debug = debug
        self.debug_test = False

        # Init coordinate transform class
        self.coord_conv = coordinate_transform()

        self.map_plotter = map_plotter(self.coord_conv)

        # Set initial values for loading of no-fly zones
        self.no_fly_zones_loaded = False
        self.no_fly_zone_reader_module = None
        self.no_fly_zone_polygons = []
        self.no_fly_zone_polygons_reduced = []
        self.no_fly_zones_reduced = False

        # Set initial values for rally points
        self.rally_points_loaded = False

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
        print test_point3d_start_UTM
        print test_point3d_goal_UTM

        self.no_fly_zone_polygons_reduced = []

        # Calculate nominal max flight time
        max_flight_distance_scaled = self.uav_nominal_battery_time_s * self.uav_nominal_airspeed_horz_mps * self.no_fly_reduce_flight_time_factor
        if self.debug:
            print colored(self.tmp_res_indent+'Max flight distance scaled %.02f [m]' % (max_flight_distance_scaled), self.default_term_color_res)

        for i in range(len(self.no_fly_zone_polygons)):
            result_start_point, dist_start_point = self.is_point_in_no_fly_zone_UTM(test_point3d_start_UTM, i, False)
            #result_goal_point, dist_goal_point = self.is_point_in_no_fly_zone_UTM(test_point3d_goal_UTM, i, False)
            #print dist_start_point
            #print dist_start_point, dist_goal_point
            #if dist_start_point <= max_flight_distance_scaled or dist_goal_point <= max_flight_distance_scaled:
            if dist_start_point <= max_flight_distance_scaled:
                self.no_fly_zone_polygons_reduced.append(self.no_fly_zone_polygons[i])

        if self.debug:
            print colored(self.tmp_res_indent+'No-fly zones / polygons reduced from %i to %i' % (len(self.no_fly_zone_polygons), len(self.no_fly_zone_polygons_reduced)), self.default_term_color_res)

        if len(self.no_fly_zone_polygons) > len(self.no_fly_zone_polygons_reduced):
            self.no_fly_zones_reduced = True
            return True
        return False

    def is_point_in_no_fly_zones_UTM(self, point3dUTM, use_buffer_zone = True):
        """
        Tests if an UTM point is within any of the no-fly zone polygons
        Input: UTM point3d as array (y, x, alt_rel ...)
        Ouput: bool (True: point is inside) and distance to nearest polygon [m]
        """
        smallest_dist = self.inf
        if self.no_fly_zones_reduced:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons_reduced
        else:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons
        for i in range(len(no_fly_zone_polygons_to_use)):
            within, dist = self.is_point_in_no_fly_zone_UTM(point3dUTM, i, use_buffer_zone)
            if dist < smallest_dist:
                smallest_dist = dist
            if within:
                return True, dist
        return False, smallest_dist
    def is_point_in_no_fly_zone_UTM(self, point3dUTM, polygon_index, use_buffer_zone = True):
        """
        Tests if an UTM point is within a specified polygon
        Input: UTM point3d as array (y, x, alt_rel ...) and polygon index
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
        if use_buffer_zone:
            if dist <= self.no_fly_zone_buffer_zone  and test_point3dUTM[2] <= self.geofence_height:
                return True, dist
            else:
                return False, dist
        else:
            if dist == 0.0  and test_point3dUTM[2] <= self.geofence_height:
                return True, dist
            else:
                return False, dist

    def is_point_in_no_fly_zones_geodetic(self, point3d_geodetic, use_buffer_zone = True):
        """
        Tests if a geodetic point is within any of the no-fly zone polygons
        Input: geodetic point3d as array (y, x, alt_rel ...)
        Ouput: bool (True: point is inside) and distance to nearest polygon [m]
        """
        smallest_dist = self.inf
        if self.no_fly_zones_reduced:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons_reduced
        else:
            no_fly_zone_polygons_to_use = self.no_fly_zone_polygons
        for i in range(len(no_fly_zone_polygons_to_use)):
            within, dist = self.is_point_in_no_fly_zone_geodetic(point3d_geodetic, i, use_buffer_zone)
            if dist < smallest_dist:
                smallest_dist = dist
            if within:
                return True, dist
        return False, smallest_dist
    def is_point_in_no_fly_zone_geodetic(self, point3d_geodetic, polygon_index, use_buffer_zone = True):
        """
        Tests if a geodetic point is within a specified polygon
        Input: geodetic point3d as array (lat, lon, alt_rel) and polygon index
        Ouput: bool (True: point is inside) and distance to polygon [m]
        """
        if isinstance(point3d_geodetic, dict):
            test_point3d_geodetic = [point3d_geodetic['lat'], point3d_geodetic['lon'], point3d_geodetic['z_rel']]
        else:
            test_point3d_geodetic = point3d_geodetic
        test_point_UTM = self.pos3d_geodetic2pos3d_UTM(test_point3d_geodetic)
        return self.is_point_in_no_fly_zone_UTM(test_point_UTM, polygon_index, use_buffer_zone)

    """ Path planning functions """
    def plan_path(self, point_start, point_goal):
        """
        Framework for different path planning algorithms
        Input: start and goal/end point (latitude, longitude in EPSG:4326, and relative altitude) as 3 value array or DICT (lat, lon, alt_rel)
        Output: planned path of geodetic points (latitude, longitude in EPSG:4326, relative altitude, and relative time)
        """
        print colored('\nPath planning started', self.default_term_color_info)

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
        print colored(self.info_alt_indent+'Start point: lat: %.03f, lon: %.03f' % (point_start_converted_geodetic['lat'], point_start_converted_geodetic['lon']), self.default_term_color_info_alt)
        print colored(self.info_alt_indent+' Goal point: lat: %.03f, lon: %.03f' % (point_goal_converted_geodetic['lat'], point_goal_converted_geodetic['lon']), self.default_term_color_info_alt)

        # Convert to UTM for path planning
        if self.debug:
            print colored('Converting points from geodetic to UTM', self.default_term_color_info)
        point_start_converted_UTM = self.coord_conv.pos3dDICT_geodetic2pos4dDICT_UTM(point_start_converted_geodetic)
        point_goal_converted_UTM = self.coord_conv.pos3dDICT_geodetic2pos4dDICT_UTM(point_goal_converted_geodetic)
        if self.debug:
            print colored(self.res_indent+'Start point: %d %c %.5fe %.5fn' % (point_start_converted_UTM['zone'], point_start_converted_UTM['letter'], point_start_converted_UTM['y'], point_start_converted_UTM['x']), self.default_term_color_res)
            print colored(self.res_indent+' Goal point: %d %c %.5fe %.5fn' % (point_goal_converted_UTM['zone'], point_goal_converted_UTM['letter'], point_goal_converted_UTM['y'], point_goal_converted_UTM['x']), self.default_term_color_res)

        # Check if start or goal point is inside a no-fly zone
        if self.is_point_in_no_fly_zones_UTM(point_start_converted_UTM)[0]:
            print colored('Start point is inside geofence', self.default_term_color_error)
            return []
        elif self.is_point_in_no_fly_zones_UTM(point_goal_converted_UTM)[0]:
            print colored('Goal point is inside geofence', self.default_term_color_error)
            return []

        # Go to selected path planner
        if self.PATH_PLANNER == self.PATH_PLANNER_ASTAR:
            print colored('Planning using A start', self.default_term_color_info)
            path_UTM = self.plan_planner_Astar(point_start_converted_UTM, point_goal_converted_UTM)
        else:
            print colored('Path planner type not defined', self.default_term_color_error)
            return []

        # Check if the path planner produced a result
        if len(path_UTM) <= 1:
            print colored('The chosen path planner could not find a solution to the problem', self.default_term_color_error)
            return []
        # Convert path from UTM to geodetic
        path_geodetic = []
        for i in range(len(path_UTM)):
            path_geodetic.append( self.coord_conv.pos4dDICT_UTM2pos4dDICT_geodetic( path_UTM[i] ) )
        return path_geodetic

    def plan_planner_Astar(self, point_start, point_goal):
        """
        A star algorithm path planner
        Input: start and goal point in UTM format
        Output: planned path of UTM points
        Inspiration:
            Wikipedia: https://en.wikipedia.org/wiki/A*_search_algorithm
            Christian Careaga: http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/
            Python docs: https://docs.python.org/2/library/heapq.html
        """
        print colored('Entered A star path planner', self.default_term_color_info)
        # Set start time of start point to 0 (relative)
        point_start['time_rel'] = 0
        # Convert points to tuples
        point_start_tuple = self.coord_conv.pos4dDICT2pos4dTUPLE_UTM(point_start)
        point_goal_tuple = self.coord_conv.pos4dDICT2pos4dTUPLE_UTM(point_goal)

        # Generate scaled neighbors based on map step size
        if len(self.neighbors_scaled) == 0:
            self.neighbors_scaled = deepcopy(self.neighbors)
            for element in range(len(self.neighbors_scaled)):
                for sub_element in range(len(self.neighbors_scaled[element])):
                    self.neighbors_scaled[element][sub_element] = self.neighbors_scaled[element][sub_element]*self.map_horz_step_size

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
        f_score[point_start_tuple] = self.heuristic_a_star(point_start_tuple, point_goal_tuple)
        if self.debug:
            print colored(self.info_alt_indent+'Start point heuristic: %f' % f_score[point_start_tuple], self.default_term_color_info_alt)

        # open list: The set of currently discovered nodes that are not evaluated yet.
        open_list = []

        # Initially, only the start node is known.
        heappush(open_list, (f_score[point_start_tuple], point_start_tuple))

        # Var for seeing the progress of the search
        open_list_popped_ctr = 0
        smallest_heuristic = self.inf
        time_start = self.get_cur_time_epoch()
        time_last = time_start

        if self.debug:
            print colored('A star initialization done, starting search', self.default_term_color_info)
        # Start searching
        while open_list and open_list_popped_ctr < self.PP_max_node_exploration: # Continue searching until the open_list is empty = all nodes discovered and evaluated; or max tries exceeded
            #time.sleep(10)
            if open_list_popped_ctr % 1000 == 0 and not open_list_popped_ctr == 0:
                time_cur = self.get_cur_time_epoch()
                print colored(self.info_alt_indent+'Visited %i nodes from the open list in %.02f [s]' % (open_list_popped_ctr, time_cur-time_last), self.default_term_color_info_alt)
                time_last = time_cur

            current = heappop(open_list)[1] # Pop (remove and get) element with lowest f score
            open_list_popped_ctr += 1 # Increment open list pop counter
            if self.print_popped_node:
                tmp_f_score = f_score[current]
                tmp_g_score = g_score.get(current)
                print colored('\n'+self.info_alt_indent+'Working on node (%.02f [m], %.02f [m], %.02f [m], %.02f [s]) with F score %.02f, G score %.02f, and heuristic %.02f' % (current[0], current[1], current[2], current[3], tmp_f_score, tmp_g_score, tmp_f_score-tmp_g_score), self.default_term_color_info)

            if self.is_goal_UTM(current, point_goal_tuple):
                time_cur = self.get_cur_time_epoch()
                print colored('Path found in %.02f [s]' % (time_cur-time_start), self.default_term_color_res)
                planned_path = self.backtrace_path(came_from, current, point_start_tuple)
                print colored('Path has %i waypoints' % (len(planned_path)), self.default_term_color_res)

                # Since the last point is within the acceptance radius of the goal point, the x, y, and z_rel of the last point is replaced with the data from the goal point and the added travel time is calculated and added
                planned_path[-1]['time_rel'] += self.calc_travel_time_from_UTMpoints(planned_path[-1], point_goal)
                planned_path[-1]['y'] = point_goal['y']
                planned_path[-1]['x'] = point_goal['x']
                planned_path[-1]['z_rel'] = point_goal['z_rel']

                # planned_path = self.reduce_path_simple_straight_line_UTM(planned_path)
                planned_path = self.reduce_path_rdp_UTM(planned_path, 5)

                # DRAW
                self.map_plotter.draw_circle_UTM(point_goal_tuple, 'green', 12)
                self.map_plotter.draw_path_UTM(planned_path)
                if self.draw_open_list:
                    self.map_plotter.draw_points_UTM(open_list, 0, 'yellow', 2)
                if self.draw_closed_list:
                    self.map_plotter.draw_points_UTM(closed_list, 1, 'grey', 5)

                return planned_path # return the path

            # Add current node to the evaluated nodes / closed list
            closed_list.add(current)


            for y, x, z in self.neighbors_scaled: # Explore the neighbors
                neighbor = self.pos4dTUPLE_UTM_copy_and_move_point(current, y, x, z) # Construct the neighbor node; the function calculates the 4 dimension by the time required to travel between the nodes

                # Calculate tentative g score by using the current g_score and the distance between the current and neighbor node
                tentative_g_score = g_score[current] + self.node_cost(current, neighbor)

                if self.print_explore_node:
                    print colored(self.info_alt_indent+'Exploring node (%.02f [m], %.02f [m], %.02f [m], %.02f [s]) with tentative G score %.02f, prevoius G score %.02f (0 = node not visited)' % (neighbor[0], neighbor[1], neighbor[2], neighbor[3], tentative_g_score, g_score.get(neighbor, 0)), self.default_term_color_info_alt)

                # Maybe test here if inside no-fly zones or violate aircrafts and skip if needed NOTE TODO see a-star-test.py
                if self.is_point_in_no_fly_zones_UTM(neighbor)[0]:
                    #print 'INSIDE GEOFENCE'
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
                    neighbor_heiristic = self.heuristic_a_star(neighbor, point_goal_tuple)
                    if neighbor_heiristic < smallest_heuristic:
                        #print colored(self.tmp_res_indent+'Smallest heuristic: %f' % neighbor_heiristic, self.default_term_color_tmp_res)
                        smallest_heuristic = neighbor_heiristic
                    f_score[neighbor] = tentative_g_score + neighbor_heiristic#self.heuristic_a_star(neighbor, point_goal_tuple) # Calculate and add the f score (combination of g score and heuristic)
                    heappush(open_list, (f_score[neighbor], neighbor)) # Add the node to the open list

        self.map_plotter.draw_circle_UTM(point_start_tuple, 'green')
        self.map_plotter.draw_circle_UTM(point_goal_tuple, 'green')
        return [] # No path found

    def heuristic_a_star(self, point4dUTM1, point4dUTM2):
        """
        Calculates the heuristic used by the A star algorithm to know how close the current node is to the goal node
        Input: 2 4d UTM points; presumably current node and goal node
        Output: scalar heuristic; current unit [m]
        """
        return self.calc_euclidian_dist_UTM(point4dUTM1, point4dUTM2)[0]

    def node_cost(self, parent_point4dUTM, child_point4dUTM):
        """
        Calculates the child node cost based on the parent node
        Input: 2 4d UTM points; 1 parent and 1 child node
        Output: scalar node cost [unitless]
        """
        # rally points
        # get the travel time which already is present in the points objects from the copy_and_move command
        travel_time = child_point4dUTM[3]-parent_point4dUTM[3]
        total_dist, horz_distance, vert_distance = self.calc_euclidian_dist_UTM(parent_point4dUTM, child_point4dUTM)
        # NOTE currently the thing below contains a linear dependency (linær afhængig)
        return 0.1*travel_time #0.1*total_dist + 0.1*travel_time

    def print_a_star_open_list_nice(self, list_in):
        print colored('\nPrinting the open list; contains %i elements' % len(list_in), self.default_term_color_info)
        for i in range(len(list_in)):
            print colored(self.info_alt_indent+'Element %i: heuristic %f, y %.02f [m], x %.02f [m], z_rel %.02f [m], time_rel %.02f ' % (i, list_in[i][0], list_in[i][1][0], list_in[i][1][1], list_in[i][1][2], list_in[i][1][3]), self.default_term_color_info_alt)
    def print_a_star_closed_list_nice(self, list_in):
        print colored('\nPrinting the closed list (set); contains %i elements' % len(list_in), self.default_term_color_info)
        itr = 0
        for element in list_in:
            print colored(self.info_alt_indent+'Element %i: y %.02f [m], x %.02f [m], z_rel %.02f [m], time_rel %.02f ' % (itr, element[0], element[1], element[2], element[3]), self.default_term_color_info_alt)
            itr += 1

    def backtrace_path(self, came_from, current_node, start_node):
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
        return path

    def reduce_path_simple_straight_line_UTM(self, planned_path):
        """
        Removes unneeded waypoints from straight lines in the planned path
        Input: UTM planned path (a set of 4d UTM tuple points)
        Output: optimized UTM planned path
        """
        dir_last = None
        index_low = 0
        index_high = 0

        out_arr = []

        for i in range(len(planned_path)-1):
            #print '\nTesting element %i' % i
            diffs = [planned_path[i+1]['y']-planned_path[i]['y'], planned_path[i+1]['x']-planned_path[i]['x'], planned_path[i+1]['z_rel']-planned_path[i]['z_rel']]
            for element in self.neighbors_scaled:
                if element == diffs:
                    dir_cur = element
                    continue
            index_high += 1
            if dir_cur != dir_last:# new path direction
                if i == 0:
                    out_arr.append(planned_path[index_low])
                else:
                    out_arr.append(planned_path[index_high])

                dir_last = dir_cur
                index_low = index_high = i
            # print 'Current direction', dir_cur
            # print ' low index: %i, high index: %i' % (index_low, index_high)

        out_arr.append(planned_path[index_high])
        out_arr.append(planned_path[-1]) # append the last element since it is not in the search

        if self.debug:
            print colored('Path reduced from %i to %i waypoints' % (len(planned_path), len(out_arr)), self.default_term_color_info)

        return out_arr

    def reduce_path_rdp_UTM(self, planned_path, tolerance=-1):
        """
        Removes waypoints according to the Ramer-Douglas-Peucker algorithm
        Input: UTM planned path (a set of 4d UTM tuple points) and optional tolerance [m]
        Output: optimized UTM planned path
        """
        print planned_path[0]
        hemisphere = planned_path[0]['hemisphere']
        zone = planned_path[0]['zone']
        letter = planned_path[0]['letter']

        planned_path_yxz = []
        for element in planned_path:
            planned_path_yxz.append([element['y'], element['x'], element['z_rel']])
        for element in planned_path_yxz:
            print element
        #{'zone': 32, 'time_rel': 0, 'hemisphere': 'N', 'letter': 'U', 'y': 589883.0749798268, 'x': 6143685.479446305, 'z_rel': 0}

        if tolerance == -1:
            print 'No tolerance provided, using tolerance = 0'
            planned_path_yxz = rdp(planned_path_yxz)
            # NOTE this is the same as the algorithm I've tried to make above
        else:
            print 'Tolerance is %.02f' % (tolerance)
            planned_path_yxz = rdp(planned_path_yxz, tolerance)

        planned_path = []
        # make the right format again
        for element in planned_path_yxz:
            print element
            #planned_path.append({'zone': z, 'time_rel': 0, 'hemisphere': 'N', 'letter': 'U', 'y': 589883.0749798268, 'x': 6143685.479446305, 'z_rel': 0})
            #{'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':element[0],'x':element[1],'z_rel':element[2],'time_rel':pos4dTUPLE[3]}

        for i in range(len(planned_path_yxz)):
            print i, planned_path_yxz[i]
            if i == 0:
                planned_path.append({'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':planned_path_yxz[i][0],'x':planned_path_yxz[i][1],'z_rel':planned_path_yxz[i][2],'time_rel':0})
            else:
                print 'Travel time:',self.calc_travel_time_from_UTMpoints(planned_path_yxz[i-1], planned_path_yxz[i]), 's'
                travel_time_abs = planned_path[i-1]['time_rel'] + self.calc_travel_time_from_UTMpoints(planned_path_yxz[i-1], planned_path_yxz[i])
                planned_path.append({'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':planned_path_yxz[i][0],'x':planned_path_yxz[i][1],'z_rel':planned_path_yxz[i][2],'time_rel':travel_time_abs})
                #TODO

        for element in planned_path:
            print element
        return planned_path

    def pos4dTUPLE_UTM_copy_and_move_point(self, point4dUTM, y_offset, x_offset, z_offset):
        """
        Takes a pos4d tuple and modifies it to a new pos4d tuple using the provided offsets
        Input: 1 4d UTM point along with 3 dimensional offset given as individual arguments
        Output: pos4dTUPLE
        """
        tmp_point_arr   = [point4dUTM[0]+y_offset, point4dUTM[1]+x_offset, point4dUTM[2]+z_offset, point4dUTM[3], point4dUTM[4], point4dUTM[5], point4dUTM[6]] # make tmp arr because it is a mutable container
        tmp_point_tuple = (tmp_point_arr[0], tmp_point_arr[1], tmp_point_arr[2], tmp_point_arr[3], tmp_point_arr[4], tmp_point_arr[5], tmp_point_arr[6]) # make tmp tuple for calculating the travel time
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
            print colored('Cannot calculate euclidian distance because formats do not match', self.default_term_color_error)
            total3d = self.inf
            vert_distance = self.inf
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
            print colored('Cannot calculate euclidian horizontal distance because formats do not match', self.default_term_color_error)
            return self.inf

    def calc_travel_time_from_UTMpoints(self, point4dUTM1, point4dUTM2):
        """
        Calculates the travel time by combining the horizontal and vertical travel times using pythagoras
        Input: 2 4d UTM points
        Output: travel time [s]
        """
        dist, horz_dist, vert_dist = self.calc_euclidian_dist_UTM(point4dUTM1, point4dUTM2)
        return ( (horz_dist / self.uav_nominal_airspeed_horz_mps) + ( vert_dist / self.uav_nominal_airspeed_vert_mps) )
    def calc_travel_time_from_dist(self, horz_dist, vert_dist):
        """
        Calculates the travel time by combining the horizontal and vertical travel times using pythagoras
        Input: horizontal and vertical distances
        Output: travel time [s]
        """
        return ( (horz_dist / self.uav_nominal_airspeed_horz_mps) + ( vert_dist / self.uav_nominal_airspeed_vert_mps) )
    def is_goal_UTM(self, point4dUTM_test, point4dUTM_goal):
        """
        Checks if the distance between the test point and the gaol points are within the allowed acceptance radius
        Input: 2 4d UTM points
        Output: True if the test point is within the acceptance radius and False if not
        """
        dist = self.calc_euclidian_horz_dist_UTM(point4dUTM_test, point4dUTM_goal)
        if dist > self.goal_acceptance_radius:
            # if self.debug:
            #     print colored(self.error_indent+'Points are NOT within goal acceptance radius; distance between points: %.02f [m], acceptance radius: %.02f [m]' % (dist, self.goal_acceptance_radius), self.default_term_color_info_alt)
            return False
        else:
            if self.debug:
                print colored(self.res_indent+'Node is within goal acceptance radius; distance between node and goal: %.02f [m], acceptance radius: %.02f [m]' % (dist, self.goal_acceptance_radius), self.default_term_color_res)
            return True

    """ Path planner evaluator functions """
    def evaluate_path(self, path):
        """
        Calculates the fitness score of a path by evaluating the different path elements
        Input: path of geodetic corrdinates
        Output: fitness score
        """
        # Check if the path is a path (more than 2 waypoints)
        if len(path) >= 2:
            print colored('\nEvaluating path', self.default_term_color_info)
            # Convert to / ensure pos4dDICT object
            path_converted = []
            try:
                tmp_var = path[0]['lat']
            except TypeError:
                print 'converting'
                for i in range(len(path)):
                    path_converted.append( self.coord_conv.pos4d2pos4dDICT_geodetic( path[i] ) )
            else:
                path_converted = path

            horz_distance, horz_distances = self.calc_horz_dist_geodetic_total(path_converted)
            vert_distance, vert_distance_ascend, vert_distance_descend, vert_distances = self.calc_vert_dist(path_converted)
            travel_time = self.calc_travel_time(path_converted)
            if self.debug:
                print colored('Calculating number of waypoints', self.default_term_color_info)
            waypoints = len(path_converted)
            if self.debug:
                print colored(self.tmp_res_indent+'Total waypoints %d' % waypoints, self.default_term_color_tmp_res)
            avg_waypoint_dist, total3d_waypoint_dist = self.calc_avg_waypoint_dist(path_converted, horz_distances, vert_distances)

            fitness = self.horz_distance_factor*horz_distance
            fitness = fitness + self.vert_distance_factor*vert_distance
            fitness = fitness + self.travel_time_factor*travel_time
            fitness = fitness + self.waypoints_factor*waypoints
            fitness = fitness + self.avg_waypoint_dist_factor*avg_waypoint_dist
            print colored('Path evaluation done', self.default_term_color_info)
            print colored(self.res_indent+'Path fitness %.02f [unitless]' % fitness, self.default_term_color_res)
            return fitness, total3d_waypoint_dist, horz_distance, vert_distance
        else:
            print colored('\nCannot evaluate path because it has fewer than 2 waypoints', self.default_term_color_error)
            return None, None, None, None

    def calc_horz_dist_geodetic_total(self, path):
        """
        Calculates the total horizontal distance from a geodetic path
        Input: path of geodetic corrdinates
        Output: total horizontal distance [m] along with a list of the horizontal distance for the individual path elements [m]
        """
        if self.debug:
            print colored('Calculating horizontal path distance (2D)', self.default_term_color_info)
        horz_distances = []
        total_horz_distance = 0 # unit: m
        for i in range(len(path)-1):
            tmp_horz_distance = self.calc_horz_dist_geodetic(path[i]['lat'], path[i]['lon'], path[i+1]['lat'], path[i+1]['lon'])
            total_horz_distance = total_horz_distance + tmp_horz_distance
            horz_distances.append(tmp_horz_distance)
        if self.debug:
            print colored(self.tmp_res_indent+'Total horizontal distance %.01f [m]' % total_horz_distance, self.default_term_color_tmp_res)
        return total_horz_distance, horz_distances
    def calc_horz_dist_geodetic(self, lat1, lon1, lat2, lon2):
        """
        Calculates the great circle distance (only horizontal, 2d) distance between 2 pairs of geodetic latitude and longitude
        Input: 2 pairs of geodetic latitude and longitude; representing 2 points
        Output: Great circle distance between points
        """
        # https://jswhit.github.io/pyproj/pyproj.Geod-class.html
        az12, az21, dist = self.geoid_distance.inv(lon1,lat1,lon2,lat2)
        return dist

    def calc_vert_dist(self, path):
        """
        Calculates the vertical distance from a geodetic path
        Input: path of geodetic corrdinates
        Output: total vertical distance (sum of abs vertical distances), vertical ascend distance, vertical descend distance, and a list of the vertical distance for the individual path elements [m] (signed)
        """
        if self.debug:
            print colored('Calculating vertical path distance (1D)', self.default_term_color_info)
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
            print colored(self.tmp_res_indent+'Total vertical distance %.01f [m], ascend distance %.01f [m], descend distance %.01f [m]' % (total_vert_distance, total_vert_distance_ascend, total_vert_distance_descend), self.default_term_color_tmp_res)
        return total_vert_distance, total_vert_distance_ascend, total_vert_distance_descend, vert_distances

    def calc_travel_time(self, path):
        """
        Calculates the travel time the timestamps in a geodetic path
        Input: path of geodetic corrdinates with timestamp (handles both relative and absoulte time)
        Output: travel time [s]
        """
        if self.debug:
            print colored('Calculating travel time', self.default_term_color_info)
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
            print colored(self.tmp_res_indent+'Travel time %.0f [s] (%s)' % (travel_time, str(datetime.timedelta(seconds=travel_time))), self.default_term_color_tmp_res)
        return travel_time

    def calc_avg_waypoint_dist(self, path, horz_distances=False, vert_distances=False):
        """
        Calculates the average distance between waypoints in a geodetic path
        Input: path of geodetic corrdinates along with optional list of horizontal and vertical distances (optimization not to calculate the great circle distances more than once)
        Output: the average waypoint distance and the total 3d distance of the path
        """
        if self.debug:
            print colored('Calculating average waypoint distance', self.default_term_color_info)
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
            print colored('Input does not match in size, average waypoint distance set to 0 [m]', self.default_term_color_error)
        if self.debug:
            print colored(self.tmp_res_indent+'Average waypoint distance %f [m]' % avg_waypoint_dist, self.default_term_color_tmp_res)
        return avg_waypoint_dist, total3d_waypoint_dist

    """ UTM """
    def utm_test(self, test_lat, test_lon):
        """
        Test of the UTM class which uses 2 geodetic points and determines the error from forward and back conversion
           Heavily inspired by https://github.com/FroboLab/frobomind/blob/master/fmLib/math/geographics/transverse_mercator/src/transverse_mercator_py/utm_test.py
        Input: 2 geodetic points
        Output: None but output in terminal
        """
        # Print initial geodetic points
        print colored('\nTest position [deg]:', self.default_term_color_info)
        print colored(self.tmp_res_indent+' latitude:  %.10f'  % (test_lat), self.default_term_color_tmp_res)
        print colored(self.tmp_res_indent+'longitude:  %.10f'  % (test_lon), self.default_term_color_tmp_res)

        # Convert from geodetic to UTM
        (hemisphere, zone, letter, easting, northing) = self.coord_conv.geodetic2UTM (test_lat,test_lon)
        print colored('Converted from geodetic to UTM [m]:', self.default_term_color_info)
        print colored(self.res_indent+'%d %c %.5fe %.5fn' % (zone, letter, easting, northing), self.default_term_color_res)

        # Convert back from UTM to geodetic
        (back_conv_lat, back_conv_lon) = self.coord_conv.UTM2geodetic (hemisphere, zone, easting, northing)
        print colored('Converted back from UTM to geodetic [deg]:', self.default_term_color_info)
        print colored(self.res_indent+' latitude:  %.10f'  % (back_conv_lat), self.default_term_color_res)
        print colored(self.res_indent+'longitude:  %.10f'  % (back_conv_lon), self.default_term_color_res)

        # Determine conversion position error [m]
        lat_err = abs(back_conv_lat-test_lat)
        lon_err = abs(back_conv_lon-test_lon)
        earth_radius = 6378137.0 # [m]
        lat_pos_err = lat_err/360.0 * 2*pi*earth_radius
        lon_pos_err = lon_err/360.0 * 2*pi*(cos(back_conv_lat)*earth_radius)
        print colored('Positional error from the two conversions [m]:', self.default_term_color_info)
        print colored(self.tmp_res_indent+' latitude:  %.09f'  % (lat_pos_err), self.default_term_color_tmp_res)
        print colored(self.tmp_res_indent+'longitude:  %.09f'  % (lon_pos_err), self.default_term_color_tmp_res)

    """ Other helper functions """
    def get_cur_time_epoch(self):
        """ Returns the current EPOCH time """
        return time.time()
    def get_cur_time_epoch_wo_us(self):
        """ Returns the current EPOCH time spripped of micro seconds """
        return round(time.time(),3)
    def get_cur_time_human_UTC(self):
        """ Returns the current UTC time in human readable format """
        return datetime.datetime.fromtimestamp( self.get_cur_time_epoch(), pytz.UTC ).strftime('%Y-%m-%d %H:%M:%S')
    def get_cur_time_human_local(self):
        """ Returns the current local time in human readable format """
        return datetime.datetime.fromtimestamp( self.get_cur_time_epoch() ).strftime('%Y-%m-%d %H:%M:%S')

    def print_path_raw(self, path):
        """ Prints the input path in a raw format """
        if len(path) >= 1:
            print colored('\n'+self.tmp_res_indent+str(path), self.default_term_color_tmp_res)
        else:
            print colored('\nCannot print path because it is empty', self.default_term_color_error)
    def print_path_nice(self, path):
        """ Prints the input path in a human readable format """
        if len(path) >= 1:
            print colored('\nPlanned path:', self.default_term_color_res)
            try:
                tmp_var = path[0]['time']
            except KeyError:
                try:
                    tmp_var = path[0]['alt']
                except KeyError:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt_rel'], path[i]['time_rel']), self.default_term_color_tmp_res)
                else:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt'], path[i]['time_rel']), self.default_term_color_tmp_res)
            else:
                try:
                    tmp_var = path[0]['alt']
                except KeyError:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt_rel'], path[i]['time']), self.default_term_color_tmp_res)
                else:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.04f [deg], lon: %.04f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt'], path[i]['time']), self.default_term_color_tmp_res)
        else:
            print colored('\nCannot print path because it is empty', self.default_term_color_error)
    def convert_rel2abs_time(self, path, start_time_epoch = False):
        """
        Converts the relative time to absoulte time by adding the current time or provided time of the start point to each point in the path
        Input: path and optional start time (all relative times are relative to the time at the starting point)
        Output: none but changes the input path and changes the key from 'time_rel' to 'time'
        """
        if len(path) > 0:
            if start_time_epoch == False:
                start_time_epoch = self.get_cur_time_epoch_wo_us()
            for i in range(len(path)):
                path[i]['time'] =path[i].pop('time_rel')
                path[i]['time'] = path[i]['time'] + start_time_epoch
    def convert_rel2abs_alt(self, path, abs_heigh_start_point):
        """
        Converts the relative heights to absoulte heights by adding the absoulute height of the start point to each point in the path
        Input: path and absoulute height of start point (all relative heights are relative to the height at the starting point)
        Output: none but changes the input path and changes the key from 'alt_rel' to 'alt'
        """
        if len(path) > 0:
            for i in range(len(path)):
                path[i]['alt'] =path[i].pop('alt_rel')

if __name__ == '__main__':
    # Save the start time before anything else
    time_task_start_s = time.time()

    # Instantiate memory_usage class
    memory_usage_module = memory_usage()
    memory_usage_module.save_heap_start() # Save the heap size before calculating and outputting statistics

    # Define start and goal points
    ## Set 1 - DO NOT use since they are actually in a no-fly zone
    #start_point_3dDICT = {'lat': 55.395774, 'lon': 10.319949, 'alt_rel': 0}
    #goal_point_3dDICT  = {'lat': 55.392968, 'lon': 10.341793, 'alt_rel': 0} # further away
    #goal_point_3dDICT  = {'lat': 55.394997, 'lon': 10.321601, 'alt_rel': 0} # closer to start point

    ## Set 2
    start_point_3dDICT = {'lat': 55.431122, 'lon': 10.420436, 'alt_rel': 0}
    goal_point_3dDICT  = {'lat': 55.427750, 'lon': 10.433737, 'alt_rel': 0}
    goal_point_3dDICT  = {'lat': 55.427203, 'lon': 10.419043, 'alt_rel': 0}

    # Instantiate UAV_path_planner class
    UAV_path_planner_module = UAV_path_planner(True)

    """ Load no-fly zones """
    print colored('Trying to load no-fly zones', UAV_path_planner_module.default_term_color_info)
    #if UAV_path_planner_module.no_fly_zone_init_and_parse(): # load online
    if UAV_path_planner_module.no_fly_zone_init_and_parse('data_sources/no_fly_zones/KmlUasZones_sec2.kml'): # load offline file
        #if UAV_path_planner_module.no_fly_zone_init_and_parse('data_sources/no_fly_zones/KmlUasZones_2018-05-02-15-50.kml'): # load offline file
        print colored('No-fly zones loaded', UAV_path_planner_module.default_term_color_res)
    else:
        print colored('No-fly zones NOT loaded', UAV_path_planner_module.default_term_color_error)

    if UAV_path_planner_module.reduce_ploygons_geodetic(start_point_3dDICT, goal_point_3dDICT):
        print colored('No-fly zones reduced', UAV_path_planner_module.default_term_color_res)
    else:
        print colored('No-fly zones NOT reduced', UAV_path_planner_module.default_term_color_error)

    """ Path planning start """
    # Plan path
    path_planned = UAV_path_planner_module.plan_path(start_point_3dDICT, goal_point_3dDICT)
    # Print planned path
    UAV_path_planner_module.print_path_nice(path_planned)
    #UAV_path_planner_module.print_path_raw(path_planned)
    # Convert the time from relative to absolute
    UAV_path_planner_module.convert_rel2abs_time(path_planned)
    # Evaluate path and also note the total distance, horizontal distance, and vertical distance for the statistics
    path_planned_fitness, total_dist, horz_dist, vert_dist = UAV_path_planner_module.evaluate_path(path_planned)

    # Show a plot of the planned path
    UAV_path_planner_module.map_plotter.show_plot()

    """ Path planning done, finalize with statistics """
    if len(path_planned) >= 1:
        time_task_end_s = time.time() # Save the end time
        memory_usage_module.save_heap() # Save the heap size before calculating and outputting statistics

        # Calculate runtime
        runtime_s = time_task_end_s - time_task_start_s

        # Calculate statistics
        used_path_planner = UAV_path_planner_module.PATH_PLANNER_NAMES[UAV_path_planner_module.PATH_PLANNER]
        estimated_flight_time = path_planned[len(path_planned)-1]['time'] - path_planned[0]['time']

        byte_amount = memory_usage_module.get_bytes()
        byte_amount_diff = memory_usage_module.get_bytes_diff()
        object_amount = memory_usage_module.get_objects()
        object_amount_diff = memory_usage_module.get_objects_diff()

        no_waypoints = len(path_planned)

        # Print statistics
        if PRINT_STATISTICS:
            print colored('\n                              Path planner: %s' % used_path_planner, 'yellow')
            print colored('                              Path fitness: %f' % path_planned_fitness, 'yellow')
            print colored('                            Total distance: %.02f [m]' % total_dist, 'yellow')
            print colored('                       Horizontal distance: %.02f [m]' % horz_dist, 'yellow')
            print colored('                         Vertical distance: %.02f [m]' % vert_dist, 'yellow')
            print colored('                                   Runtime: %f [s] (%s)' % (runtime_s, str(datetime.timedelta(seconds=runtime_s))), 'yellow')
            print colored('                      Number of bytes used: %i' % byte_amount, 'yellow')
            print colored('  Number of bytes used by the path planner: %i' % byte_amount_diff, 'yellow')
            print colored('                    Number of objects used: %i' % object_amount, 'yellow')
            print colored('Number of objects used by the path planner: %i' % object_amount_diff, 'yellow')
            print colored('                     Estimated flight time: %.02f [s] (%s)' % (estimated_flight_time, str(datetime.timedelta(seconds=estimated_flight_time))), 'yellow') # TODO
            print colored('                       Number of waypoints: %i' % (no_waypoints), 'yellow')
            print colored('                                      Path: %s' % (str(path_planned)), 'yellow')

        # Save statistics to file
        if SAVE_STATISTICS_TO_FILE:
            now = datetime.datetime.now()
            file_name = ('PP_%d-%02d-%02d-%02d-%02d.csv' % (now.year, now.month, now.day, now.hour, now.minute))
            sub_folder = 'results'
            file_name = sub_folder+'/'+file_name
            output_file_CSV = open(file_name, 'w')
            output_writer_CSV = csv.writer(output_file_CSV,quoting=csv.QUOTE_MINIMAL)
            fields = ['path planner', 'path fitness [unitless]', 'total distance [m]', 'horizontal distance [m]', 'vertical distance [m]', 'runtime [s]', 'bytes used', 'bytes used by the path planner', 'objects used', 'objects used by the path planner', 'flight time [s]', 'waypoints', 'start point', 'goal point', 'path']
            output_writer_CSV.writerow(fields)
            data = [used_path_planner, path_planned_fitness, total_dist, horz_dist, vert_dist, runtime_s, byte_amount, byte_amount_diff, object_amount, object_amount_diff, estimated_flight_time, no_waypoints, start_point_3dDICT, goal_point_3dDICT, path_planned]
            output_writer_CSV.writerow(data)
            output_file_CSV.close()
