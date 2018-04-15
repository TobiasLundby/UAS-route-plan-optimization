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

""" Program defines """
PATH_PLANNER_ASTAR = 0

""" User defines """
default_plot_color = "red"
default_plot_alpha = 0.8
PATH_PLANNER = PATH_PLANNER_ASTAR
horz_distance_factor = 1
vert_distance_factor = 1
travel_time_factor = 1
waypoints_factor = 1
avg_waypoint_dist_factor = 1

def merc(lat, lon):
    r_major = 6378137.000
    x = r_major * math.radians(lon)
    scale = x/lon
    y = 180.0/math.pi * math.log(math.tan(math.pi/4.0 + lat * (math.pi/180.0)/2.0)) * scale
    return (x, y)

def geo_point(lat, lon = None):
    if isinstance(lat, list) and lon == None:
        lon = lat[1]
        lat = lat[0]
    x,y = merc(lat, lon)
    return (lat,lon,x,y)

def geo_points(arr_in):
    return_arr = []
    for element in arr_in:
        # print element
        tmp_geo_point = geo_point(element)
        return_arr.append([tmp_geo_point[0], tmp_geo_point[1], tmp_geo_point[2], tmp_geo_point[3]])
    return return_arr

""" Plot functions """
def generate_circle(plot, geo_point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
    plot.circle(x=geo_point[2], y=geo_point[3], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

def generate_line(plot, geo_point_start, geo_point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
    plot.line([geo_point_start[2], geo_point_end[2]], [geo_point_start[3], geo_point_end[3]], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)

def generate_path(plot, geo_points_in):
    if len(geo_points_in) > 0:
        if len(geo_points_in[0]) == 2: # convert points to mercator system if not already converted
            #print "Converting points"
            geo_points_in = geo_points(geo_points_in)
        for i in range(len(geo_points_in)-1):
            generate_line(plot, geo_points_in[i], geo_points_in[i+1])
        for i in range(len(geo_points_in)):
            if i == 0 or i == (len(geo_points_in)-1): # first point make other colored
                generate_circle(plot, geo_points_in[i], 'firebrick')
            else:
                generate_circle(plot, geo_points_in[i], 'red',7)

def generate_geofence(plot, geofence_in):
    if len(geofence_in) > 2:
        if len(geofence_in[0]) == 2: # convert points to mercator system if not already converted
            print "Converting points"
            geofence_in = geo_points(geofence_in)
        patch_merc_points_x = []
        patch_merc_points_y = []
        for i in range(len(geofence_in)):
            patch_merc_points_x.append(geofence_in[i][2])
            patch_merc_points_y.append(geofence_in[i][3])
        plot.patch(patch_merc_points_x, patch_merc_points_y, alpha=0.5, line_width=2)

""" Path planning functions """
def plan_path(geo_point_start, geo_point_end):
    # Convert points to proper representation
    if PATH_PLANNER == PATH_PLANNER_ASTAR:
        print "Planning using A start"
        path = plan_planner_Astar(geo_point_start, geo_point_end)
    else:
        print "Path planner type not defined"
        return []
    return path

def plan_planner_Astar(geo_point_start, geo_point_end):
    print "Entered A star path planner"
    print "Start point: %f, %f" % (geo_point_start[0], geo_point_start[1])
    print "End point: %f, %f" % (geo_point_end[0], geo_point_end[1])
    return_arr = []
    return_arr.append(geo_point_start)
    return_arr.append(geo_point_end)
    return return_arr

""" Path planner evaluator functions """
def evaluate_path(path):
    print "Evaluating path"
    horz_distance = calc_horz_dist(path)
    vert_distance = calc_vert_dist(path)
    travel_time = calc_travel_time(path)
    waypoints = len(path)
    avg_waypoint_dist = calc_avg_waypoint_dist(path)

    fitness = horz_distance_factor*horz_distance
    fitness = fitness + vert_distance_factor*vert_distance
    fitness = fitness + travel_time_factor*travel_time
    fitness = fitness + waypoints_factor*waypoints
    fitness = fitness + avg_waypoint_dist_factor*avg_waypoint_dist
    return fitness

def calc_horz_dist(path):
    print "Calculating horizontal path distance"
    horz_distance = 0 # unit: m
    # code
    print "Horizontal distance %f [m]" % horz_distance
    return horz_distance

def calc_vert_dist(path):
    print "Calculating vertical path distance"
    vert_distance = 0 # unit: m
    # code
    print "Vertical distance %f [m]" % vert_distance
    return vert_distance

def calc_travel_time(path):
    print "Calculating travel time"
    travel_time = 0 # unit: s
    # code
    print "Travel time %f [s]" % travel_time
    return travel_time

def calc_avg_waypoint_dist(path):
    print "Calculating average waypoint distance"
    avg_waypoint_dist = 0 # unit: m
    # code
    # choose which distance metric is used or use function which reacts based on defines.
    print "Average waypoint distance %f [m]" % avg_waypoint_dist
    return avg_waypoint_dist

if __name__ == '__main__':
    output_file("tile.html")

    use_bounds = False
    bounds_bottom_left = geo_point(54.5,8)
    bounds_top_right = geo_point(58,13)
    print "Bottom left: lat: %d, lng: %d, x: %d, y: %d" % (bounds_bottom_left)
    print "  Top right: lat: %d, lng: %d, x: %d, y: %d" % (bounds_top_right)

    # range bounds supplied in web mercator coordinates
    if use_bounds:
        p = figure(x_range=(bounds_bottom_left[2], bounds_top_right[2]), y_range=(bounds_bottom_left[3], bounds_top_right[3]))#, x_axis_type="mercator", y_axis_type="mercator")
    else:
        p = figure()
    # alternative way to add lat lng axis due to above commented method does not work, https://github.com/bokeh/bokeh/issues/6986
    p.xaxis[0].ticker = MercatorTicker(dimension='lon')
    p.yaxis[0].ticker = MercatorTicker(dimension='lat')
    p.xaxis[0].formatter = MercatorTickFormatter(dimension='lon')
    p.yaxis[0].formatter = MercatorTickFormatter(dimension='lat')
    p.add_tile(CARTODBPOSITRON)
    #p.add_tile(STAMEN_TERRAIN) # remember to import when needed

    # source = ColumnDataSource(
    #     data=dict(lat=[ 30.29,  30.20,  30.29],
    #               lon=[-97.70, -97.74, -97.78])
    # )
    #p.circle(x="lon", y="lat", size=15, fill_color="blue", fill_alpha=0.8, source=source)

    #tmp_val2 = geo_point(55.400, 10.366)
    #generate_circle(p, tmp_val2)
    #tmp_geo_point = geo_point(55.500, 10.396)
    #generate_circle(p, tmp_geo_point)

    #generate_line(p, tmp_val2, tmp_geo_point)

    #points = [[55.400, 10.366],[55.500, 10.396],[55.450, 10.376]]
    points = [[55.397, 10.319949],[55.41, 10.349689],[55.391653, 10.352]]
    points_geofence = [[55.395774, 10.319949],[55.406105, 10.349689],[55.391653, 10.349174], [55.392968, 10.341793], [55.386873, 10.329691]]
    #converted_points = geo_points(points)

    generate_path(p, points)
    generate_geofence(p, points_geofence)

    print plan_path(points[0], points[len(points)-1])
    print evaluate_path(points)

    #show(p)
