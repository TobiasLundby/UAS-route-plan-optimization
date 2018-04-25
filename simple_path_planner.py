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
    # Path evaluation (path fitness) factors
    horz_distance_factor = 1
    vert_distance_factor = 1
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

    def geo_point(self, lat, lon = None):
        if isinstance(lat, list) and lon == None:
            lon = lat[1]
            lat = lat[0]
        x,y = self.Geodetic2PseudoMercator(lat, lon)
        return (lat,lon,x,y)

    def geo_points(self, arr_in):
        return_arr = []
        for element in arr_in:
            # print element
            tmp_geo_point = self.geo_point(element)
            return_arr.append([tmp_geo_point[0], tmp_geo_point[1], tmp_geo_point[2], tmp_geo_point[3]])
        return return_arr

    """ Plot functions """
    def generate_circle(self, plot, geo_point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        plot.circle(x=geo_point[2], y=geo_point[3], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

    def generate_line(self, plot, geo_point_start, geo_point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
        plot.line([geo_point_start[2], geo_point_end[2]], [geo_point_start[3], geo_point_end[3]], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)

    def generate_path(self, plot, geo_points_in):
        if len(geo_points_in) > 0:
            if len(geo_points_in[0]) == 3: # convert points to mercator system if not already converted
                #print "Converting points"
                geo_points_in = self.geo_points(geo_points_in)
            for i in range(len(geo_points_in)-1):
                self.generate_line(plot, geo_points_in[i], geo_points_in[i+1])
            for i in range(len(geo_points_in)):
                if i == 0 or i == (len(geo_points_in)-1): # first point make other colored
                    self.generate_circle(plot, geo_points_in[i], 'firebrick')
                else:
                    self.generate_circle(plot, geo_points_in[i], 'red',7)

    def generate_geofence(self, plot, geofence_in):
        if len(geofence_in) > 2:
            if len(geofence_in[0]) == 2: # convert points to mercator system if not already converted
                print "Converting points"
                geofence_in = self.geo_points(geofence_in)
            patch_merc_points_x = []
            patch_merc_points_y = []
            for i in range(len(geofence_in)):
                patch_merc_points_x.append(geofence_in[i][2])
                patch_merc_points_y.append(geofence_in[i][3])
            plot.patch(patch_merc_points_x, patch_merc_points_y, alpha=0.5, line_width=2)

    def pos3d2pos3dDICT(self, pos3d):
        """
        Input: 3 value 1 element array of lat, lon, alt
        Output: 3 value 1 element DICT of lat, lon, alt
        """
        return {'lat':pos3d[0],'lon':pos3d[1],'alt':pos3d[2]}

    def pos3dDICT2pos3d(self, pos3dDICT):
        """
        Input: 3 value 1 element DICT of lat, lon, alt
        Output: 3 value 1 element array of lat, lon, alt
        """
        return [pos3pos3dDICT['lat'],pos3pos3dDICT['lon'],pos3pos3dDICT['alt']]
    def pos4d2pos4dDICT(self, pos4d):
        """
        Input: 4 value 1 element array of lat, lon, alt, time
        Output: 4 value 1 element DICT of lat, lon, alt, time
        """
        return {'lat':pos4d[0],'lon':pos4d[1],'alt':pos4d[2],'time':pos4d[2]}
    def pos4dDICT2pos4d(self, pos4dDICT):
        """
        Input: 4 value 1 element DICT of lat, lon, alt, time
        Output: 4 value 1 element array of lat, lon, alt, time
        """
        return [pos4dDICT['lat'],pos4dDICT['lon'],pos4dDICT['alt'],pos4dDICT['time']]

    """ Path planning functions """
    def plan_path(self, geo_point_start, geo_point_end):
        """
        Input: start and goal/end point (latitude, longitude in EPSG:4326, and altitude) as 3 value array or DICT (lat, lon, alt)
        Output: planned path of points (latitude, longitude in EPSG:4326, altitude, and time)
        """
        # Convert to pos3dDICT
        try:
            tmp_var = geo_point_start['lat']
        except TypeError:
            geo_point_start = self.pos3d2pos3dDICT(geo_point_start)
        try:
            tmp_var = geo_point_end['lat']
        except TypeError:
            geo_point_end = self.pos3d2pos3dDICT(geo_point_end)


        print colored('Path planning started', self.default_term_color_info)
        # Convert points to proper representation
        if PATH_PLANNER == PATH_PLANNER_ASTAR:
            print colored('Planning using A start', self.default_term_color_info)
            path = self.plan_planner_Astar(geo_point_start, geo_point_end)
        else:
            print colored('Path planner type not defined', self.default_term_color_error)
            return []
        return path

    def plan_planner_Astar(self, geo_point_start, geo_point_end):
        print colored('Entered A star path planner', self.default_term_color_info)
        print colored('Start point: lat: %f, lon: %f' % (geo_point_start['lat'], geo_point_start['lon']), self.default_term_color_tmp_res)
        print colored('End point: lat: %f, lon: %f' % (geo_point_end['lat'], geo_point_end['lon']), self.default_term_color_tmp_res)
        return_arr = []
        return_arr.append(geo_point_start)
        return_arr.append(geo_point_end)
        return return_arr

    """ Path planner evaluator functions """
    def evaluate_path(self, path):
        print colored('Evaluating path', self.default_term_color_info)
        horz_distance = self.calc_horz_dist_geodetic_total(path)
        vert_distance = self.calc_vert_dist(path)
        travel_time = self.calc_travel_time(path)
        waypoints = len(path)
        if self.debug:
            print colored('Total waypoints %d' % waypoints, self.default_term_color_tmp_res)
        avg_waypoint_dist = self.calc_avg_waypoint_dist(path)

        fitness = self.horz_distance_factor*horz_distance
        fitness = fitness + self.vert_distance_factor*vert_distance
        fitness = fitness + self.travel_time_factor*travel_time
        fitness = fitness + self.waypoints_factor*waypoints
        fitness = fitness + self.avg_waypoint_dist_factor*avg_waypoint_dist
        if self.debug:
            print colored('Path fitness %f [m]' % fitness, self.default_term_color_tmp_res)
        return fitness

    def calc_horz_dist_geodetic_total(self, path):
        print colored('Calculating horizontal path distance (2D)', self.default_term_color_info)
        total_horz_distance = 0 # unit: m
        for i in range(len(path)-1):
            total_horz_distance = total_horz_distance + self.calc_horz_dist_geodetic(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
        if self.debug:
            print colored('Total horizontal distance %f [m]' % total_horz_distance, self.default_term_color_tmp_res)
        return total_horz_distance
    def calc_horz_dist_geodetic(self, lat1, lon1, lat2, lon2):
        # https://jswhit.github.io/pyproj/pyproj.Geod-class.html
        az12, az21, dist = geoid_distance.inv(lon1,lat1,lon2,lat2)
        return dist

    def calc_vert_dist(self, path):
        print colored('Calculating vertical path distance (1D)', self.default_term_color_info)
        vert_distance = 0 # unit: m
        # code
        if self.debug:
            print colored('Vertical distance %f [m]' % vert_distance, self.default_term_color_tmp_res)
        return vert_distance

    def calc_travel_time(self, path):
        print colored('Calculating travel time', self.default_term_color_info)
        travel_time = 0 # unit: s
        # code
        if self.debug:
            print colored('Travel time %f [s]' % travel_time, self.default_term_color_tmp_res)
        return travel_time

    def calc_avg_waypoint_dist(self, path, total_path_length=False):
        print colored('Calculating average waypoint distance', self.default_term_color_info)
        # if total_path_length == False:
        #     total_path_length = self.calc_horz_dist_geodetic_total
        avg_waypoint_dist = 0 # unit: m
        # code
        # choose which distance metric is used or use function which reacts based on defines.
        if self.debug:
            print colored('Average waypoint distance %f [m]' % avg_waypoint_dist, self.default_term_color_tmp_res)
        return avg_waypoint_dist

if __name__ == '__main__':
    # Set output file for Bokeh
    output_file("path_planning.html")

    UAV_path_planner_module = UAV_path_planner(True)

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

    points_DICT = [
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


    UAV_path_planner_module.generate_path(p, points)
    UAV_path_planner_module.generate_geofence(p, points_geofence)

    path_planned = UAV_path_planner_module.plan_path(points[0], points[len(points)-1])
    path_planned_fitness = UAV_path_planner_module.evaluate_path(points)

    #print UAV_path_planner_module.calc_horz_dist_geodetic(points[0][0],points[0][1],points[1][0],points[1][1]), "m"

    #UAV_path_planner_module.calc_horz_dist_geodetic_total(points_geofence)

    #show(p)
