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

default_plot_color = "red"
default_plot_alpha = 0.8

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


def generate_circle(plot, geo_point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
    plot.circle(x=geo_point[2], y=geo_point[3], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

def generate_line(plot, geo_point_start, geo_point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
    plot.line([geo_point_start[2], geo_point_end[2]], [geo_point_start[3], geo_point_end[3]], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)

def generate_path(plot, geo_points_in):
    if len(geo_points_in) > 0:
        if len(geo_points_in[0]) == 2: # convert points to mercator system if not already converted
            print "Converting points"
            geo_points_in = geo_points(geo_points_in)
        for i in range(len(geo_points_in)-1):
            generate_line(plot, geo_points_in[i], geo_points_in[i+1])
        for i in range(len(geo_points_in)):
            generate_circle(plot, geo_points_in[i])

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

points = [[55.400, 10.366],[55.500, 10.396],[55.450, 10.376]]
converted_points = geo_points(points)

generate_path(p, converted_points)

show(p)
