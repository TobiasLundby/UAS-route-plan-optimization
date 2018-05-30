#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: plot various objects on an OpenStreetMap plot using bokeh
    License: BSD 3-Clause
"""

""" Import libraries """
from bokeh.plotting import figure, show, output_file
from bokeh.tile_providers import CARTODBPOSITRON
from bokeh.models import ColumnDataSource, MercatorTicker, MercatorTickFormatter

""" User defines """
default_term_color_res  = 'green'
default_plot_color      = 'red'
default_plot_alpha      = 0.8

class map_plotter():
    def __init__(self, coord_conv_object, output_filename = "path_planning.html", debug = False):
        """
        Init method
        Input: optional debug parameter which toogles debug messages
        Output: none
        """
        self.debug = debug

        self.coord_conv = coord_conv_object

        # Set output file for Bokeh
        output_file(output_filename)

        # Set bounds for map plot
        use_bounds = False

        # range bounds supplied in web mercator coordinates
        if use_bounds:
            bounds_bottom_left_Geodetic = [54.5, 8]
            bounds_top_right_Geodetic = [58, 13]
            bounds_bottom_left_PseudoMercator = self.coord_conv.Geodetic2PseudoMercator(bounds_bottom_left_Geodetic)
            bounds_top_right_PseudoMercator   = self.coord_conv.Geodetic2PseudoMercator(bounds_top_right_Geodetic)
            print colored('Bottom left: lat: %d, lng: %d, x: %d, y: %d' % (bounds_bottom_left_Geodetic[0], bounds_bottom_left_Geodetic[1], bounds_bottom_left_PseudoMercator[0], bounds_bottom_left_PseudoMercator[1]), 'yellow')
            print colored('  Top right: lat: %d, lng: %d, x: %d, y: %d' % (bounds_top_right_Geodetic[0], bounds_top_right_Geodetic[1], bounds_top_right_PseudoMercator[0], bounds_top_right_PseudoMercator[1]), 'yellow')
            self.p = figure(x_range=(bounds_bottom_left_PseudoMercator[0], bounds_top_right_PseudoMercator[0]), y_range=(bounds_bottom_left_PseudoMercator[1], bounds_top_right_PseudoMercator[1]))#, x_axis_type="mercator", y_axis_type="mercator")
        else:
            self.p = figure()
        # alternative way to add lat lng axis due to above commented method does not work, https://github.com/bokeh/bokeh/issues/6986
        self.p.xaxis[0].ticker = MercatorTicker(dimension='lon')
        self.p.yaxis[0].ticker = MercatorTicker(dimension='lat')
        self.p.xaxis[0].formatter = MercatorTickFormatter(dimension='lon')
        self.p.yaxis[0].formatter = MercatorTickFormatter(dimension='lat')
        self.p.add_tile(CARTODBPOSITRON)
        # Add axis labels to plot
        self.p.xaxis.axis_label = "Longitude [deg]"
        self.p.yaxis.axis_label = "Latitude [deg]"

        self.cur_path_color = 0
        self.path_end_colors = ['firebrick', 'seagreen', 'steelblue', 'fuchsia', 'magenta']
        self.path_colors = ['red', 'green', 'blue', 'deeppink', 'purple']


    def draw_circle_OSM(self, point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        """
        Draws a circle on the map plot from OSM coordinates
        Input: 2d OSM DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        self.p.circle(x=point['x'], y=point['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)
    def draw_circle_geodetic(self, point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        """
        Draws a circle on the map plot from geodetic coordinates
        Input: 2d geodetic DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        points_in_converted = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(point)
        self.p.circle(x=points_in_converted['x'], y=points_in_converted['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)
    def draw_circle_UTM(self, point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        """
        Draws a circle on the map plot from UTM coordinates
        Input: 4d UTM DICT or tuple point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        if isinstance(point, dict):
            (back_conv_lat, back_conv_lon) = self.coord_conv.UTM2geodetic(point['hemisphere'], point['zone'], point['y'], point['x'])
        elif isinstance(point, tuple):
            (back_conv_lat, back_conv_lon) = self.coord_conv.UTM2geodetic(point[4], point[5], point[0], point[1])
        point2dDICT = {'lat':back_conv_lat,'lon':back_conv_lon}
        points_in_converted = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT)
        self.p.circle(x=points_in_converted['x'], y=points_in_converted['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

    def draw_points_UTM(self, points_in, list_type, point_color = default_plot_color, point_size = 7):
        """
        Draws points in the closed or open list
        Input: list and list type (0: open list, 1: closed list)
        Output: none but drawing on the provided plot
        """
        if list_type == 0:
            print colored(self.info_alt_indent+'Drawing points from open list; elements % i' % ( len(points_in) ), self.default_term_color_info_alt)
            for element in points_in:
                self.draw_circle_UTM(element[1], point_color, point_size)
        elif list_type == 1:
            print colored(self.info_alt_indent+'Drawing points from closed list; elements % i' % ( len(points_in) ), self.default_term_color_info_alt)
            for element in points_in:
                self.draw_circle_UTM(element, point_color, point_size)

    def draw_line_OSM(self, point_start, point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
        """
        Draws a line on the map plot from OSM coordinates
        Input: 2d OSM DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        self.p.line([point_start['x'], point_end['x']], [point_start['y'], point_end['y']], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)
    def draw_line_UTM(self, point_start, point_end, line_color_in = default_plot_color, line_width_in = 2, line_alpha_in = default_plot_alpha):
        """
        Draws a line on the map plot from UTM coordinates
        Input: 2d (also accepts 4d) UTM DICT or tuple point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        # Convert start point from UTM to geodetic to OSM
        if isinstance(point_start, dict):
            (back_conv_lat_start, back_conv_lon_start) = self.coord_conv.UTM2geodetic(point_start['hemisphere'], point_start['zone'], point_start['y'], point_start['x'])
        elif isinstance(point_start, tuple):
            (back_conv_lat_start, back_conv_lon_start) = self.coord_conv.UTM2geodetic(point_start[4], point_start[5], point_start[0], point_start[1])
        point2dDICT_start = {'lat':back_conv_lat_start,'lon':back_conv_lon_start}
        points_in_converted_start = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT_start)
        # Convert end point from UTM to geodetic to OSM
        if isinstance(point_end, dict):
            (back_conv_lat_end, back_conv_lon_end) = self.coord_conv.UTM2geodetic(point_end['hemisphere'], point_end['zone'], point_end['y'], point_end['x'])
        elif isinstance(point_end, tuple):
            (back_conv_lat_end, back_conv_lon_end) = self.coord_conv.UTM2geodetic(point_end[4], point_end[5], point_end[0], point_end[1])
        point2dDICT_end = {'lat':back_conv_lat_end,'lon':back_conv_lon_end}
        points_in_converted_end = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT_end)
        # Draw line from OSM points
        self.p.line([points_in_converted_start['x'], points_in_converted_end['x']], [points_in_converted_start['y'], points_in_converted_end['y']], line_color = line_color_in, line_width = line_width_in, line_alpha = line_alpha_in)

    def draw_path_geodetic(self, points_in):
        """
        Draws a path on the map plot from geodetic coordinates
        Input: set of 2d geodetic points (either array or DICT)
        Output: none but drawing on the provided plot
        """
        if len(points_in) > 0:
            # Check and convert points
            points_in_converted = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(points_in)

            for i in range(len(points_in_converted)-1):
                self.draw_line_OSM(points_in_converted[i], points_in_converted[i+1], line_color_in = self.path_colors[self.cur_path_color])
            for i in range(len(points_in_converted)):
                if i == 0 or i == (len(points_in_converted)-1): # first point make other colored
                    self.draw_circle_OSM(points_in_converted[i], self.path_end_colors[self.cur_path_color])
                else:
                    self.draw_circle_OSM(points_in_converted[i], self.path_colors[self.cur_path_color] ,7)
            self.cur_path_color += 1
            if self.cur_path_color >= len(self.path_colors):
                self.cur_path_color = 0

    def draw_path_UTM(self, points_in):
        """
        Draws a path on the map plot from UTM coordinates
        Input: set of 4d UTM point tuples
        Output: none but drawing on the provided plot
        """
        if len(points_in) > 0:
            for i in range(len(points_in)-1):
                self.draw_line_UTM(points_in[i], points_in[i+1], line_color_in = self.path_colors[self.cur_path_color])
            for i in range(len(points_in)):
                if i == 0 or i == (len(points_in)-1): # first point make other colored
                    self.draw_circle_UTM(points_in[i], self.path_end_colors[self.cur_path_color])
                else:
                    self.draw_circle_UTM(points_in[i], self.path_colors[self.cur_path_color],7)
            self.cur_path_color += 1
            if self.cur_path_color >= len(self.path_colors):
                self.cur_path_color = 0

    def draw_geofence_geodetic(self, geofence_in):
        """
        Draws a ploygon on the map plot from geodetic coordinates
        Input: a set of 2d geodetic points (either array or DICT) representing a polygon
        Output: none but drawing on the provided plot
        """
        if len(geofence_in) > 2:
            # Check and convert points
            geofence_in_converted = self.coord_conv.check_pos2dALL_geodetic2pos2dDICT_OSM(geofence_in)

            points_OSM_x = []
            points_OSM_y = []
            for i in range(len(geofence_in)):
                points_OSM_x.append(geofence_in_converted[i]['x'])
                points_OSM_y.append(geofence_in_converted[i]['y'])
            self.p.patch(points_OSM_x, points_OSM_y, alpha=0.5, line_width=2)

    def show_plot(self):
        show(self.p)
