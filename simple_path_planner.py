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

from bokeh.plotting import figure, show, output_file
from bokeh.tile_providers import CARTODBPOSITRON
from bokeh.models import ColumnDataSource, MercatorTicker, MercatorTickFormatter
import math
from termcolor import colored
from pyproj import Proj, transform, Geod
import datetime

""" Program defines """
PATH_PLANNER_ASTAR = 0
geodetic_proj = Proj('+init=EPSG:4326')  # EPSG:3857 - WGS 84 / Pseudo-Mercator - https://epsg.io/3857 - OSM projection
OSM_proj = Proj('+init=EPSG:3857')  # EPSG:4326 - WGS 84 / World Geodetic System 1984, used in GPS - https://epsg.io/4326
geoid_distance = Geod(ellps='WGS84')

""" User defines """
default_term_color_res = 'green'
default_plot_color = 'red'
default_plot_alpha = 0.8
PATH_PLANNER = PATH_PLANNER_ASTAR

class UAV_path_planner():
    # Terminal output colors
    default_term_color_info = 'cyan'
    default_term_color_error = 'red'
    default_term_color_tmp_res = 'yellow'
    tmp_res_indent = ' -> '
    # Path evaluation (path fitness) factors
    horz_distance_factor = 1
    vert_distance_factor = 2
    travel_time_factor = 1
    waypoints_factor = 1
    avg_waypoint_dist_factor = 1
    # Other
    geofence_height = 100 # unit: m
    def __init__(self, debug = False):
        self.debug = debug

    def Geodetic2PseudoMercator(self, lat, lon = False):
        """
        Input: latitude, longitude (EPSG:4326) or array of lat and lon as the lat input
        Output: x, y(EPSG:3857)
        """
        if lon == False and len(lat) == 2:
            x,y = transform(geodetic_proj, OSM_proj, lat[1], lat[0])
        else:
            x,y = transform(geodetic_proj, OSM_proj, lon, lat)
        return (x,y)
    def PseudoMercator2Geodetic(self, x,y):
        """
        Input: x, y(EPSG:3857)
        Output: latitude, longitude (EPSG:4326)
        """
        lon, lat = transform(OSM_proj, geodetic_proj, x, y)
        return (lat, lon)

    """ Plot functions """
    def draw_circle(self, plot, point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        """
        Input: plot and a 2d DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        plot.circle(x=point['x'], y=point['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

    def draw_line(self, plot, point_start, point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
        """
        Input: plot and a 2d DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        plot.line([point_start['x'], point_end['x']], [point_start['y'], point_end['y']], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)

    def draw_path(self, plot, points_in):
        """
        Input: plot and a set of 2d points (either array or DICT)
        Output: none but drawing on the provided plot
        """
        if len(points_in) > 0:
            # Check and convert points
            points_in_converted = self.check_pos2dALL_geodetic2pos2dDICT_OSM(points_in)

            for i in range(len(points_in_converted)-1):
                self.draw_line(plot, points_in_converted[i], points_in_converted[i+1])
            for i in range(len(points_in_converted)):
                if i == 0 or i == (len(points_in_converted)-1): # first point make other colored
                    self.draw_circle(plot, points_in_converted[i], 'firebrick')
                else:
                    self.draw_circle(plot, points_in_converted[i], 'red',7)

    def generate_geofence(self, plot, geofence_in):
        """
        Input: plot and a set of 2d points (either array or DICT) representing a polygon
        Output: none but drawing on the provided plot
        """
        if len(geofence_in) > 2:
            # Check and convert points
            geofence_in_converted = self.check_pos2dALL_geodetic2pos2dDICT_OSM(geofence_in)

            points_OSM_x = []
            points_OSM_y = []
            for i in range(len(geofence_in)):
                points_OSM_x.append(geofence_in_converted[i]['x'])
                points_OSM_y.append(geofence_in_converted[i]['y'])
            plot.patch(points_OSM_x, points_OSM_y, alpha=0.5, line_width=2)

    def check_pos2dALL_geodetic2pos2dDICT_OSM(self, pos2dALL_geodetic):
        pos2dDICT_OSM = []
        try:
            tmp_var = pos2dALL_geodetic[0]['lat']
        except TypeError:
            # convert the points
            for i in range(len(pos2dALL_geodetic)):
                pos2dDICT_OSM.append( self.pos2dDICT_geodetic2pos2dDICT_OSM( self.pos2d2pos2dDICT_geodetic(pos2dALL_geodetic[i]) ) )
        else:
            for i in range(len(pos2dALL_geodetic)):
                pos2dDICT_OSM.append( self.pos2dDICT_geodetic2pos2dDICT_OSM(pos2dALL_geodetic[i]) )
        return pos2dDICT_OSM

    def pos2dDICT_geodetic2pos2dDICT_OSM(self, pos2dDICT_geodetic):
        x,y = self.Geodetic2PseudoMercator(pos2dDICT_geodetic['lat'], pos2dDICT_geodetic['lon'])
        return {'x':x,'y':y}
    def pos2dDICT_OSM2pos2dDICT_geodetic(self, pos2dDICT_OSM):
        lat,lon = self.PseudoMercator2Geodetic(pos2dDICT_OSM['x'], pos2dDICT_OSM['y'])
        return {'lat':lat,'lon':lon}

    def pos2d2pos2dDICT_OSM(self, pos2d):
        """
        Input: 2 value 1 element array of x, y
        Output: 2 value 1 element DICT of x, y
        """
        return {'x':pos3d[0],'y':pos3d[1]}
    def pos2dDICT2pos2d_OSM(self, pos2dDICT):
        """
        Input: 2 value 1 element array of x, y
        Output: 2 value 1 element DICT of x, y
        """
        return [pos2dDICT['x'],pos2dDICT['y']]

    def pos2d2pos2dDICT_geodetic(self, pos2d):
        """
        Input: 2 value 1 element array of lat, lon
        Output: 2 value 1 element DICT of lat, lon
        """
        return {'lat':pos2d[0],'lon':pos2d[1]}
    def pos2dDICT2pos2d_geodetic(self, pos3dDICT):
        """
        Input: 2 value 1 element DICT of lat, lon
        Output: 2 value 1 element array of lat, lon
        """
        return [pos2dDICT['lat'],pos2dDICT['lon']]

    def pos3d2pos3dDICT_geodetic(self, pos3d):
        """
        Input: 3 value 1 element array of lat, lon, alt
        Output: 3 value 1 element DICT of lat, lon, alt
        """
        return {'lat':pos3d[0],'lon':pos3d[1],'alt':pos3d[2]}
    def pos3dDICT2pos3d_geodetic(self, pos3dDICT):
        """
        Input: 3 value 1 element DICT of lat, lon, alt
        Output: 3 value 1 element array of lat, lon, alt
        """
        return [pos3dDICT['lat'],pos3dDICT['lon'],pos3dDICT['alt']]

    def pos4d2pos4dDICT_geodetic(self, pos4d):
        """
        Input: 4 value 1 element array of lat, lon, alt, time
        Output: 4 value 1 element DICT of lat, lon, alt, time
        """
        return {'lat':pos4d[0],'lon':pos4d[1],'alt':pos4d[2],'time':pos4d[3]}
    def pos4dDICT2pos4d_geodetic(self, pos4dDICT):
        """
        Input: 4 value 1 element DICT of lat, lon, alt, time
        Output: 4 value 1 element array of lat, lon, alt, time
        """
        return [pos4dDICT['lat'],pos4dDICT['lon'],pos4dDICT['alt'],pos4dDICT['time']]

    """ Path planning functions """
    def plan_path(self, point_start, point_end):
        """
        Input: start and goal/end point (latitude, longitude in EPSG:4326, and altitude) as 3 value array or DICT (lat, lon, alt)
        Output: planned path of points (latitude, longitude in EPSG:4326, altitude, and time)
        """
        # Convert to pos3dDICT
        try:
            tmp_var = point_start['lat']
        except TypeError:
            point_start_converted = self.pos3d2pos3dDICT_geodetic(point_start)
        else:
            point_start_converted = point_start
        try:
            tmp_var = point_end['lat']
        except TypeError:
            point_end_converted = self.pos3d2pos3dDICT_geodetic(point_end)
        else:
            point_end_converted = point_end

        # NOTE: convert to UTM for path planning

        print colored('Path planning started', self.default_term_color_info)
        # Go to selected path planner
        if PATH_PLANNER == PATH_PLANNER_ASTAR:
            print colored('Planning using A start', self.default_term_color_info)
            path = self.plan_planner_Astar(point_start_converted, point_end_converted)
        else:
            print colored('Path planner type not defined', self.default_term_color_error)
            return []
        return path

    def plan_planner_Astar(self, point_start, point_end):
        print colored('Entered A star path planner', self.default_term_color_info)
        print colored('Start point: lat: %.03f, lon: %.03f' % (point_start['lat'], point_start['lon']), self.default_term_color_tmp_res)
        print colored('End point: lat: %.03f, lon: %.03f' % (point_end['lat'], point_end['lon']), self.default_term_color_tmp_res)
        return_arr = []
        return_arr.append(point_start)
        return_arr.append(point_end)
        return return_arr

    """ Path planner evaluator functions """
    def evaluate_path(self, path):
        print colored('Evaluating path', self.default_term_color_info)
        # Convert to / ensure pos4dDICT object
        path_converted = []
        try:
            tmp_var = path[0]['lat']
        except TypeError:
            for i in range(len(path)):
                path_converted.append( self.pos4d2pos4dDICT_geodetic( path[i] ) )
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
        avg_waypoint_dist = self.calc_avg_waypoint_dist(path_converted, horz_distances, vert_distances)

        fitness = self.horz_distance_factor*horz_distance
        fitness = fitness + self.vert_distance_factor*vert_distance
        fitness = fitness + self.travel_time_factor*travel_time
        fitness = fitness + self.waypoints_factor*waypoints
        fitness = fitness + self.avg_waypoint_dist_factor*avg_waypoint_dist
        print colored('Path evaluation done', self.default_term_color_info)
        print colored(self.tmp_res_indent+'Path fitness %.02f [unitless]' % fitness, self.default_term_color_tmp_res)
        return fitness

    def calc_horz_dist_geodetic_total(self, path):
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
        # https://jswhit.github.io/pyproj/pyproj.Geod-class.html
        az12, az21, dist = geoid_distance.inv(lon1,lat1,lon2,lat2)
        return dist

    def calc_vert_dist(self, path):
        if self.debug:
            print colored('Calculating vertical path distance (1D)', self.default_term_color_info)
        vert_distances = []
        total_vert_distance = 0 # unit: m
        total_vert_distance_ascend = 0 # unit: m
        total_vert_distance_descend = 0 # unit: m
        for i in range(len(path)-1):
            tmp_vert_distance = path[i+1]['alt']-path[i]['alt']
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
        if self.debug:
            print colored('Calculating travel time', self.default_term_color_info)
        travel_time = 0 # unit: s
        for i in range(len(path)-1):
            travel_time = travel_time + (path[i+1]['time'] - path[i]['time'])
        if self.debug:
            print colored(self.tmp_res_indent+'Travel time %.0f [s] (%s)' % (travel_time, str(datetime.timedelta(seconds=travel_time))), self.default_term_color_tmp_res)
        return travel_time

    def calc_avg_waypoint_dist(self, path, horz_distances=False, vert_distances=False):
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
                total3d_waypoint_dist = total3d_waypoint_dist + math.sqrt( math.pow(horz_distances[i], 2) + math.pow(vert_distances[i], 2) )
            avg_waypoint_dist = total3d_waypoint_dist / len(horz_distances)
        else:
            print colored('Input does not match in size, average waypoint distance set to 0 [m]', self.default_term_color_error)
        if self.debug:
            print colored(self.tmp_res_indent+'Average waypoint distance %f [m]' % avg_waypoint_dist, self.default_term_color_tmp_res)
        return avg_waypoint_dist

if __name__ == '__main__':
    # Set output file for Bokeh
    output_file("path_planning.html")

    UAV_path_planner_module = UAV_path_planner(False)

    # Set bounds for map plot
    use_bounds = False

    # range bounds supplied in web mercator coordinates
    if use_bounds:
        bounds_bottom_left_Geodetic = [54.5, 8]
        bounds_top_right_Geodetic = [58, 13]
        bounds_bottom_left_PseudoMercator = UAV_path_planner_module.Geodetic2PseudoMercator(bounds_bottom_left_Geodetic)
        bounds_top_right_PseudoMercator   = UAV_path_planner_module.Geodetic2PseudoMercator(bounds_top_right_Geodetic)
        print colored('Bottom left: lat: %d, lng: %d, x: %d, y: %d' % (bounds_bottom_left_Geodetic[0], bounds_bottom_left_Geodetic[1], bounds_bottom_left_PseudoMercator[0], bounds_bottom_left_PseudoMercator[1]), 'yellow')
        print colored('  Top right: lat: %d, lng: %d, x: %d, y: %d' % (bounds_top_right_Geodetic[0], bounds_top_right_Geodetic[1], bounds_top_right_PseudoMercator[0], bounds_top_right_PseudoMercator[1]), 'yellow')
        p = figure(x_range=(bounds_bottom_left_PseudoMercator[0], bounds_top_right_PseudoMercator[0]), y_range=(bounds_bottom_left_PseudoMercator[1], bounds_top_right_PseudoMercator[1]))#, x_axis_type="mercator", y_axis_type="mercator")
    else:
        p = figure()
    # alternative way to add lat lng axis due to above commented method does not work, https://github.com/bokeh/bokeh/issues/6986
    p.xaxis[0].ticker = MercatorTicker(dimension='lon')
    p.yaxis[0].ticker = MercatorTicker(dimension='lat')
    p.xaxis[0].formatter = MercatorTickFormatter(dimension='lon')
    p.yaxis[0].formatter = MercatorTickFormatter(dimension='lat')
    p.add_tile(CARTODBPOSITRON)
    # Add axis labels to plot
    p.xaxis.axis_label = "Longitude [deg]"
    p.yaxis.axis_label = "Latitude [deg]"


    points = [[55.397, 10.319949, 0],[55.41, 10.349689,20],[55.391653, 10.352,0]]
    points_geofence = [[55.395774, 10.319949],[55.406105, 10.349689],[55.391653, 10.349174], [55.392968, 10.341793], [55.386873, 10.329691]]

    points2d_DICT = [
        {
            'lat': 55.395774,
            'lon': 10.319949
        },
        {
            'lat': 55.406105,
            'lon': 10.349689
        },
        {
            'lat': 55.391653,
            'lon': 10.349174
        },
        {
            'lat': 55.392968,
            'lon': 10.341793
        }
    ]
    points4d_DICT = [
        {
            'lat': 55.395774,
            'lon': 10.319949,
            'alt': 0,
            'time': 1524663524
        },
        {
            'lat': 55.406105,
            'lon': 10.349689,
            'alt': 20,
            'time': 1524663584
        },
        {
            'lat': 55.391653,
            'lon': 10.349174,
            'alt': 20,
            'time': 1524663624
        },
        {
            'lat': 55.392968,
            'lon': 10.341793,
            'alt': 0,
            'time': 1524663684
        }
    ]
    points4d = [
        [
            55.395774,
            10.319949,
            0,
            1524663524
        ],
        [
            55.406105,
            10.349689,
            20,
            1524663584
        ],
        [
            55.391653,
            10.349174,
            20,
            1524663624
        ],
        [
            55.392968,
            10.341793,
            0,
            1524663684
        ]
    ]


    UAV_path_planner_module.draw_path(p, points2d_DICT)
    UAV_path_planner_module.generate_geofence(p, points_geofence)

    path_planned = UAV_path_planner_module.plan_path(points[0], points[len(points)-1])
    path_planned_fitness = UAV_path_planner_module.evaluate_path(points4d)

    #print UAV_path_planner_module.calc_horz_dist_geodetic(points[0][0],points[0][1],points[1][0],points[1][1]), "m"

    #UAV_path_planner_module.calc_horz_dist_geodetic_total(points_geofence)

    #show(p)
