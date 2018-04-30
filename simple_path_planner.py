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
from bokeh.plotting import figure, show, output_file
from bokeh.tile_providers import CARTODBPOSITRON
from bokeh.models import ColumnDataSource, MercatorTicker, MercatorTickFormatter
from math import pow, sqrt, pi, cos
from termcolor import colored
from pyproj import Proj, transform, Geod
import datetime
from libs.transverse_mercator_py.utm import utmconv
import time
import datetime # datetime.now
import pytz # timezones in the datetime format
from copy import copy, deepcopy
from guppy import hpy # for getting heap info
import csv # for saving statistics and path

""" Program defines """
PATH_PLANNER_ASTAR      = 0
geodetic_proj           = Proj('+init=EPSG:4326')  # EPSG:3857 - WGS 84 / Pseudo-Mercator - https://epsg.io/3857 - OSM projection
OSM_proj                = Proj('+init=EPSG:3857')  # EPSG:4326 - WGS 84 / World Geodetic System 1984, used in GPS - https://epsg.io/4326
geoid_distance          = Geod(ellps='WGS84') # used for calculating Great Circle Distance

""" User defines """
default_term_color_res  = 'green'
default_plot_color      = 'red'
default_plot_alpha      = 0.8
PATH_PLANNER            = PATH_PLANNER_ASTAR
PATH_PLANNER_NAMES      = ['A star algorithm']
PRINT_STATISTICS        = False
SAVE_STATISTICS_TO_FILE = False

class UAV_path_planner():
    """ UAV constants """
    uav_nominal_airspeed_horz_mps = 15 # unit: m/s
    uav_nominal_airspeed_vert_mps = 5 # unit: m/s
    """ Path planning constants """
    goal_acceptance_radius = 0.5 # unit: m
    map_step_size = 0.5 # unit: m
    # neighbors in 8 connect 2d planning; values can be scaled if needed
    # neighbors = [ [0,1], [0,-1], [1,0], [-1,0], [1,1], [1,-1], [-1,1], [-1,-1] ]
    # neighbors in 4 connect 2d planning; values can be scaled if needed
    neighbors = [ [0,1], [0,-1], [1,0], [-1,0] ]
    neighbors_scaled = []
    """ Terminal output colors """
    default_term_color_info = 'cyan'
    default_term_color_info_alt = 'magenta'
    info_alt_indent = ' -- '
    default_term_color_error = 'red'
    error_indent = ' ÷÷ '
    default_term_color_tmp_res = 'yellow'
    tmp_res_indent = ' -> '
    default_term_color_res = 'green'
    res_indent = ' ++ '
    """ Path evaluation (path fitness) factors """
    horz_distance_factor = 1 # unit: unitless
    vert_distance_factor = 2 # unit: unitless
    travel_time_factor = 1 # unit: unitless
    waypoints_factor = 1 # unit: unitless
    avg_waypoint_dist_factor = 1 # unit: unitless
    """ Other """
    geofence_height = 100 # unit: m
    forever = 60*60*24*365*100  # unit: s; 100 years excl. leap year
    inf = 4294967295 # 32bit from 0

    def __init__(self, debug = False):
        """ Constructor """
        self.debug = debug

        # Instantiate utmconv class
        self.uc = utmconv()

        # Set output file for Bokeh
        output_file("path_planning.html")

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

    def Geodetic2PseudoMercator(self, lat, lon = False):
        """
        Converts geodetic latitude and longitude to pseudo mercator (OSM) x and y
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
        Converts x and y pseudo mercator (OSM) to geodetic latitude and longitude
        Input: x, y(EPSG:3857)
        Output: latitude, longitude (EPSG:4326)
        """
        lon, lat = transform(OSM_proj, geodetic_proj, x, y)
        return (lat, lon)

    """ Plot functions """
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
        points_in_converted = self.check_pos2dALL_geodetic2pos2dDICT_OSM(point)
        self.p.circle(x=points_in_converted['x'], y=points_in_converted['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)
    def draw_circle_UTM(self, point, circle_color_in = default_plot_color, circle_size_in = 10, circle_alpha_in = default_plot_alpha):
        """
        Draws a circle on the map plot from UTM coordinates
        Input: 4d UTM DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic(point['hemisphere'], point['zone'], point['y'], point['x'])
        point2dDICT = {'lat':back_conv_lat,'lon':back_conv_lon}
        points_in_converted = self.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT)
        self.p.circle(x=points_in_converted['x'], y=points_in_converted['y'], size = circle_size_in, fill_color=circle_color_in, fill_alpha=circle_alpha_in)

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
        Input: 2d (also accepts 4d) UTM DICT point along with optional graphic parameters
        Output: none but drawing on the provided plot
        """
        # Convert start point from UTM to geodetic to OSM
        (back_conv_lat_start, back_conv_lon_start) = self.uc.utm_to_geodetic(point_start['hemisphere'], point_start['zone'], point_start['y'], point_start['x'])
        point2dDICT_start = {'lat':back_conv_lat_start,'lon':back_conv_lon_start}
        points_in_converted_start = self.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT_start)
        # Convert end point from UTM to geodetic to OSM
        (back_conv_lat_end, back_conv_lon_end) = self.uc.utm_to_geodetic(point_end['hemisphere'], point_end['zone'], point_end['y'], point_end['x'])
        point2dDICT_end = {'lat':back_conv_lat_end,'lon':back_conv_lon_end}
        points_in_converted_end = self.check_pos2dALL_geodetic2pos2dDICT_OSM(point2dDICT_end)
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
            points_in_converted = self.check_pos2dALL_geodetic2pos2dDICT_OSM(points_in)

            for i in range(len(points_in_converted)-1):
                self.draw_line_OSM(points_in_converted[i], points_in_converted[i+1])
            for i in range(len(points_in_converted)):
                if i == 0 or i == (len(points_in_converted)-1): # first point make other colored
                    self.draw_circle_OSM(points_in_converted[i], 'firebrick')
                else:
                    self.draw_circle_OSM(points_in_converted[i], 'red',7)

    def draw_geofence_geodetic(self, geofence_in):
        """
        Draws a ploygon on the map plot from geodetic coordinates
        Input: a set of 2d geodetic points (either array or DICT) representing a polygon
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
            self.p.patch(points_OSM_x, points_OSM_y, alpha=0.5, line_width=2)

    def show_plot(self):
        show(self.p)

    """
    Geographical location formats:
        pos2d
            Geodetic: 2 value 1 element array of lat, lon
            OSM: 2 value 1 element array of x, y

        pos2dDICT:
            Geodetic: 2 value 1 element DICT of lat, lon; geodetic
            OSM: 2 value 1 element OSM DICT of x, y

        pos3d:
            Geodetic: 3 value 1 element array of lat, lon, alt_rel

        pos3dDICT:
            Geodetic: 3 value 1 element DICT of lat, lon, alt_rel

        pos4d:
            UTM: 7 value 1 element array of y (easting), x (norting), alt_rel, time_rel, hemisphere, zone, letter
            Geodetic: 4 value 1 element array of lat, lon, alt_rel, time_rel

        pos4dDICT:
            UTM: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
            Geodetic:
                4 value 1 element DICT of lat, lon, alt_rel, time_rel
                4 value 1 element DICT of lat, lon, alt, time_rel

        Fields:
            lat: latitude [dd]
            lon: longitude [dd]
            alt_rel: relative altitude to start point height [m]
            alt: (absolute) GPS altitude [m]
            time_rel: relative time to start point time (default 0) [s]
            time: (absolute) timestamps in EPOCH format [s]
            x:
                OSM: positive right on map
                UTM: norting, positive up on map
            y:
                OSM: positive up on map
                UTM: easting, positive right on map
    """
    def check_pos2dALL_geodetic2pos2dDICT_OSM(self, pos2dALL_geodetic):
        """ Cheks and converts all formats of pos2D (geodetic) to pos2dDICT_OSM
        Input: 2 value 1 element DICT of lat, lon or array of lat, lon
        Output: 2 value 1 element OSM DICT of x, y
        """
        if isinstance(pos2dALL_geodetic, list):
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
        elif isinstance(pos2dALL_geodetic, dict):
            pos2dDICT_OSM = self.pos2dDICT_geodetic2pos2dDICT_OSM(pos2dALL_geodetic)
        return pos2dDICT_OSM

    def pos2dDICT_geodetic2pos2dDICT_OSM(self, pos2dDICT_geodetic):
        """ Converts pos2dDICT_geodetic to pos2dDICT_OSM
        Input: 2 value 1 element DICT of lat, lon
        Output: 2 value 1 element OSM DICT of x, y
        """
        x,y = self.Geodetic2PseudoMercator(pos2dDICT_geodetic['lat'], pos2dDICT_geodetic['lon'])
        return {'x':x,'y':y}
    def pos2dDICT_OSM2pos2dDICT_geodetic(self, pos2dDICT_OSM):
        """ Converts pos2dDICT_OSM to pos2dDICT_geodetic
        Input: 2 value 1 element OSM DICT of x, y
        Output: 2 value 1 element DICT of lat, lon
        """
        lat,lon = self.PseudoMercator2Geodetic(pos2dDICT_OSM['x'], pos2dDICT_OSM['y'])
        return {'lat':lat,'lon':lon}

    def pos2d2pos2dDICT_OSM(self, pos2d):
        """ Converts pos2d to pos2dDICT, OSM
        Input: 2 value 1 element array of x, y
        Output: 2 value 1 element DICT of x, y
        """
        return {'x':pos2d[0],'y':pos2d[1]}
    def pos2dDICT2pos2d_OSM(self, pos2dDICT):
        """ Converts pos2d to pos2d, OSM
        Input: 2 value 1 element array of x, y
        Output: 2 value 1 element DICT of x, y
        """
        return [pos2dDICT['x'],pos2dDICT['y']]

    def pos2d2pos2dDICT_geodetic(self, pos2d):
        """ Converts pos2d to pos2dDICT, geodetic
        Input: 2 value 1 element array of lat, lon
        Output: 2 value 1 element DICT of lat, lon
        """
        return {'lat':pos2d[0],'lon':pos2d[1]}
    def pos2dDICT2pos2d_geodetic(self, pos2dDICT):
        """ Converts pos2dDICT to pos2d, geodetic
        Input: 2 value 1 element DICT of lat, lon
        Output: 2 value 1 element array of lat, lon
        """
        return [pos2dDICT['lat'],pos2dDICT['lon']]

    def pos3dDICT2pos2dDICT_geodetic(self, pos3dDICT):
        """ Converts pos3dDICT to pos2dDICT, geodetic
        Input: 3 value 1 element DICT of lat, lon, alt_rel
        Output: 2 value 1 element DICT of lat, lon; discards alt_rel
        """
        return {'lat':pos3dDICT['lat'],'lon':pos3dDICT['lon']}
    def pos2dDICT2pos3dDICT_geodetic(self, pos2dDICT):
        """ Converts pos2dDICT to pos3dDICT, geodetic
        Input: 2 value 1 element DICT of lat, lon
        Output: 3 value 1 element DICT of lat, lon, alt_rel (defaulted to 0)
        """
        return {'lat':pos2dDICT['lat'],'lon':pos2dDICT['lon'],'alt_rel':0}

    def pos3d2pos3dDICT_geodetic(self, pos3d):
        """ Converts pos3d to pos3dDICT, geodetic
        Input: 3 value 1 element array of lat, lon, alt_rel
        Output: 3 value 1 element DICT of lat, lon, alt_rel
        """
        return {'lat':pos3d[0],'lon':pos3d[1],'alt_rel':pos3d[2]}
    def pos3dDICT2pos3d_geodetic(self, pos3dDICT):
        """ Converts pos3dDICT to pos3d, geodetic
        Input: 3 value 1 element DICT of lat, lon, alt_rel
        Output: 3 value 1 element array of lat, lon, alt_rel
        """
        return [pos3dDICT['lat'],pos3dDICT['lon'],pos3dDICT['alt_rel']]

    def pos3dDICT_geodetic2pos4dDICT_UTM(self, pos3dDICT):
        """ Converts pos3dDICT (geodetic) to pos4dDICT (UTM)
        Input: 3 value 1 element DICT of lat, lon, alt_rel
        Output: 4 value 1 element DICT of x, y, z_rel (relative to start height), time_rel (defaulted to None and relative to start time) along with UTM parameters (hemisphere, zone, and letter)
        See https://developer.dji.com/mobile-sdk/documentation/introduction/flightController_concepts.html for coordinate system reference
        and https://www.basicairdata.eu/knowledge-center/background-topics/coordinate-system/
            x = norting = up: flying direction
            y = easting = right: tangent to flying direction
        """
        (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3dDICT['lat'],pos3dDICT['lon'])
        return {'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':easting,'x':northing,'z_rel':pos3dDICT['alt_rel'],'time_rel':None}
    def pos4dDICT_UTM2pos4dDICT_geodetic(self, pos4dDICT_UTM):
        """ Converts pos4dDICT (UTM) to pos4dDICT (geodetic)
        Input: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        Output: 4 value 1 element DICT of lat, lon, alt_rel (relative to start height), time_rel
        """
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic (pos4dDICT_UTM['hemisphere'], pos4dDICT_UTM['zone'], pos4dDICT_UTM['y'], pos4dDICT_UTM['x'])
        return {'lat':back_conv_lat,'lon':back_conv_lon,'alt_rel':pos4dDICT_UTM['z_rel'],'time_rel':pos4dDICT_UTM['time_rel']}

    def pos4d2pos4dDICT_geodetic(self, pos4d):
        """ Converts pos4d to pos4dDICT, geodetic
        Input: 4 value 1 element array of lat, lon, alt_rel, time_rel
        Output: 4 value 1 element DICT of lat, lon, alt_rel, time_rel
        """
        return {'lat':pos4d[0],'lon':pos4d[1],'alt_rel':pos4d[2],'time_rel':pos4d[3]}
    def pos4dDICT2pos4d_geodetic(self, pos4dDICT):
        """ Converts pos4dDICT to pos4d, geodetic
        Input: 4 value 1 element DICT of lat, lon, alt_rel, time_rel
        Output: 4 value 1 element array of lat, lon, alt_rel, time_rel
        """
        return [pos4dDICT['lat'],pos4dDICT['lon'],pos4dDICT['alt_rel'],pos4dDICT['time_rel']]

    def pos4d2pos4dDICT_UTM(self, pos4d):
        """ Converts pos4d to pos4dDICT, UTM
        Input: 7 value 1 element array of y (easting), x (norting), alt_rel, time_rel, hemisphere, zone, letter
        Output: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        """
        return {'hemisphere':pos4d[4],'zone':pos4d[5],'letter':pos4d[6],'y':pos4d[0],'x':pos4d[1],'z_rel':pos4d[2],'time_rel':pos4d[3]}
    def pos4dDICT2pos4d_UTM(self, pos4dDICT):
        """ Converts pos4d to pos4dDICT, UTM
        Input: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        Output: 7 value 1 element array of y (easting), x (norting), alt_rel, time_rel, hemisphere, zone, letter
        """
        return [pos4dDICT['y'], pos4dDICT['x'], pos4dDICT['z_rel'], pos4dDICT['time_rel'], pos4dDICT['hemisphere'], pos4dDICT['zone'], pos4dDICT['letter']]

    def pos4dDICT2pos4dTUPLE_UTM(self, pos4dDICT):
        """ Converts pos4d to pos4dDICT, UTM
        Input: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        Output: 7 value 1 element TUPLE of y (easting), x (norting), alt_rel, time_rel, hemisphere, zone, letter
        """
        return (pos4dDICT['y'], pos4dDICT['x'], pos4dDICT['z_rel'], pos4dDICT['time_rel'], pos4dDICT['hemisphere'], pos4dDICT['zone'], pos4dDICT['letter'])
    def pos4dTUPLE2pos4dDICT_UTM(self, pos4dTUPLE):
        """ Converts pos4d to pos4dDICT, UTM
        Input: 7 value 1 element array of y (easting), x (norting), alt_rel, time_rel, hemisphere, zone, letter
        Output: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        """
        return {'hemisphere':pos4dTUPLE[4],'zone':pos4dTUPLE[5],'letter':pos4dTUPLE[6],'y':pos4dTUPLE[0],'x':pos4dTUPLE[1],'z_rel':pos4dTUPLE[2],'time_rel':pos4dTUPLE[3]}

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
            point_start_converted_geodetic = self.pos3d2pos3dDICT_geodetic(point_start)
        else:
            point_start_converted_geodetic = point_start
        try:
            tmp_var = point_goal['lat']
        except TypeError:
            point_goal_converted_geodetic = self.pos3d2pos3dDICT_geodetic(point_goal)
        else:
            point_goal_converted_geodetic = point_goal
        print colored(self.info_alt_indent+'Start point: lat: %.03f, lon: %.03f' % (point_start_converted_geodetic['lat'], point_start_converted_geodetic['lon']), self.default_term_color_info_alt)
        print colored(self.info_alt_indent+' Goal point: lat: %.03f, lon: %.03f' % (point_goal_converted_geodetic['lat'], point_goal_converted_geodetic['lon']), self.default_term_color_info_alt)

        # Convert to UTM for path planning
        if self.debug:
            print colored('Converting points from geodetic to UTM', self.default_term_color_info)
        point_start_converted_UTM = self.pos3dDICT_geodetic2pos4dDICT_UTM(point_start_converted_geodetic)
        point_goal_converted_UTM = self.pos3dDICT_geodetic2pos4dDICT_UTM(point_goal_converted_geodetic)
        if self.debug:
            print colored(self.res_indent+'Start point: %d %c %.5fe %.5fn' % (point_start_converted_UTM['zone'], point_start_converted_UTM['letter'], point_start_converted_UTM['y'], point_start_converted_UTM['x']), self.default_term_color_res)
            print colored(self.res_indent+' Goal point: %d %c %.5fe %.5fn' % (point_goal_converted_UTM['zone'], point_goal_converted_UTM['letter'], point_goal_converted_UTM['y'], point_goal_converted_UTM['x']), self.default_term_color_res)

        # Go to selected path planner
        if PATH_PLANNER == PATH_PLANNER_ASTAR:
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
            path_geodetic.append( self.pos4dDICT_UTM2pos4dDICT_geodetic( path_UTM[i] ) )
        return path_geodetic

    def plan_planner_Astar(self, point_start, point_goal):
        """
        A star algorithm path planner
        Input: start and goal point in UTM format
        Output: planned path of UTM points
        """
        print colored('Entered A star path planner', self.default_term_color_info)

        # Generate scaled neighbors based on map step size
        if len(self.neighbors_scaled) == 0:
            self.neighbors_scaled = deepcopy(self.neighbors)
            for element in range(len(self.neighbors_scaled)):
                for sub_element in range(len(self.neighbors_scaled[element])):
                    self.neighbors_scaled[element][sub_element] = self.neighbors_scaled[element][sub_element]*self.map_step_size

        # closed list: The set of nodes already evaluated
        closed_list = set()

        # For each node, which node it can most efficiently be reached from.
        # If a node can be reached from many nodes, cameFrom will eventually contain the
        # most efficient previous step.
        came_from = {}

        print point_start
        point_start_arr = self.pos4dDICT2pos4d_UTM(point_start)
        print point_start_arr
        point_start_tuple = self.pos4dDICT2pos4dTUPLE_UTM(point_start)
        print point_start_tuple
        point_start_back_DICT = self.pos4dTUPLE2pos4dDICT_UTM(point_start_tuple)
        print point_start_back_DICT

        # For each node, the cost of getting from the start node to that node.
        gscore = {}
        # The cost of going from start to start is zero.
        gscore[point_start_tuple] = 0
        print gscore

        # For each node, the total cost of getting from the start node to the goal
        # by passing by that node. That value is partly known, partly heuristic.
        fscore = {}
        # For the first node, that value is completely heuristic.
        fscore[point_start] = self.heuristic_a_star(point_start, point_goal)
        if self.debug:
            print colored(self.info_alt_indent+'Start point heuristic: %f' % fscore[point_start], self.default_term_color_info_alt)
        print fscore
        # open list: The set of currently discovered nodes that are not evaluated yet.
        open_list = []

        # Initially, only the start node is known.
        heappush(open_list, (fscore[point_start], point_start))

        path = []
        # NOTE: the 4 lines below are purely for testing
        # point_start['time_rel'] = 0
        # point_goal['time_rel'] = 20
        # path.append(point_start)
        # path.append(point_goal)
        return path

    def heuristic_a_star(self, point4dUTM1, point4dUTM2):
        return 10

    def calc_euclidian_dist_UTM(self, point4dUTM1, point4dUTM2):
        """
        Calculates the euclidian distance (3d) between 2 UTM points
        Input: 2 UTM points
        Output: euclidian distance, euclidian horizontal distance (2d), and vertical distance (1d)
        """
        total3d = sqrt( pow(point4dUTM1['x']-point4dUTM2['x'],2) + pow(point4dUTM1['y']-point4dUTM2['y'],2) + pow(point4dUTM1['z_rel']-point4dUTM2['z_rel'],2) )
        horz_distance = self.calc_euclidian_horz_dist_UTM(point4dUTM1, point4dUTM2)
        vert_distance = abs(point4dUTM1['z_rel']-point4dUTM2['z_rel'])
        return total3d, horz_distance, vert_distance

    def calc_euclidian_horz_dist_UTM(self, point4dUTM1, point4dUTM2):
        """
        Calculates the horizontal euclidian distance (2d) between 2 UTM points
        Input: 2 UTM points
        Output: horizontal euclidian distance
        """
        return sqrt( pow(point4dUTM1['x']-point4dUTM2['x'],2) + pow(point4dUTM1['y']-point4dUTM2['y'],2) )

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
            if self.debug:
                print colored(self.error_indent+'Points are NOT within goal acceptance radius; distance between points: %.02f [m], acceptance radius: %.02f [m]' % (dist, self.goal_acceptance_radius), self.default_term_color_info_alt)
            return False
        else:
            if self.debug:
                print colored(self.res_indent+'Points are within goal acceptance radius; distance between points: %.02f [m], acceptance radius: %.02f [m]' % (dist, self.goal_acceptance_radius), self.default_term_color_res)
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
        az12, az21, dist = geoid_distance.inv(lon1,lat1,lon2,lat2)
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
        (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm (test_lat,test_lon)
        print colored('Converted from geodetic to UTM [m]:', self.default_term_color_info)
        print colored(self.res_indent+'%d %c %.5fe %.5fn' % (zone, letter, easting, northing), self.default_term_color_res)

        # Convert back from UTM to geodetic
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic (hemisphere, zone, easting, northing)
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
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.03f [deg], lon: %.03f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt_rel'], path[i]['time_rel']), self.default_term_color_tmp_res)
                else:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.03f [deg], lon: %.03f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt'], path[i]['time_rel']), self.default_term_color_tmp_res)
            else:
                try:
                    tmp_var = path[0]['alt']
                except KeyError:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.03f [deg], lon: %.03f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt_rel'], path[i]['time']), self.default_term_color_tmp_res)
                else:
                    for i in range(len(path)):
                        print colored(self.tmp_res_indent+'Waypoint %d: lat: %.03f [deg], lon: %.03f [deg], alt: %.01f [m], time: %.02f [s]' %(i, path[i]['lat'], path[i]['lon'], path[i]['alt'], path[i]['time']), self.default_term_color_tmp_res)
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

    # Instantiate UAV_path_planner class
    UAV_path_planner_module = UAV_path_planner(True)

    """ Define some points for testing """
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
            'alt_rel': 0,
            'time_rel': 1524663524
        },
        {
            'lat': 55.406105,
            'lon': 10.349689,
            'alt_rel': 20,
            'time_rel': 1524663584
        },
        {
            'lat': 55.391653,
            'lon': 10.349174,
            'alt_rel': 20,
            'time_rel': 1524663624
        },
        {
            'lat': 55.392968,
            'lon': 10.341793,
            'alt_rel': 0,
            'time_rel': 1524663684
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


    #UAV_path_planner_module.draw_path_geodetic(points2d_DICT)
    #UAV_path_planner_module.draw_geofence_geodetic(points_geofence)

    """ Path planning start """
    # Define start and goal points
    start_point_3dDICT = {'lat': 55.395774, 'lon': 10.319949, 'alt_rel': 0}
    goal_point_3dDICT  = {'lat': 55.392968, 'lon': 10.341793, 'alt_rel': 0}
    # Plan path
    path_planned = UAV_path_planner_module.plan_path(start_point_3dDICT, goal_point_3dDICT)
    # Print planned path
    UAV_path_planner_module.print_path_nice(path_planned)
    UAV_path_planner_module.print_path_raw(path_planned)
    # Convert the time from relative to absolute
    UAV_path_planner_module.convert_rel2abs_time(path_planned)
    # Evaluate path and also note the total distance, horizontal distance, and vertical distance for the statistics
    path_planned_fitness, total_dist, horz_dist, vert_dist = UAV_path_planner_module.evaluate_path(path_planned)

    # Show a plot of the planned path
    #UAV_path_planner_module.show_plot()

    """ Path planning done, finalize with statistics """
    if len(path_planned) >= 1:
        # Save the end time
        time_task_end_s = time.time()
        # Save the heap size before calculating and outputting statistics
        h = hpy()
        heap = h.heap() # Extract heap
        heap_str = str(heap) # Convert to string to make a deepcopy so it is not just pointing to an object which changes
        # Calculate runtime
        runtime_s = time_task_end_s - time_task_start_s

        # Calculate statistics
        used_path_planner = PATH_PLANNER_NAMES[PATH_PLANNER]
        estimated_flight_time = path_planned[len(path_planned)-1]['time'] - path_planned[0]['time']

        index_end_objects = heap_str.find(' objects')
        object_amount = int(heap_str[22:index_end_objects])

        index_start_bytes = heap_str.find('Total size = ')
        index_end_bytes = heap_str.find(' bytes.')
        byte_amount = int(heap_str[index_start_bytes+13:index_end_bytes])

        no_waypoints = len(path_planned)

        # Print statistics
        if PRINT_STATISTICS:
            print colored('\n          Path planner: %s' % used_path_planner, 'yellow')
            print colored('          Path fitness: %f' % path_planned_fitness, 'yellow')
            print colored('        Total distance: %.02f [m]' % total_dist, 'yellow')
            print colored('   Horizontal distance: %.02f [m]' % horz_dist, 'yellow')
            print colored('     Vertical distance: %.02f [m]' % vert_dist, 'yellow')
            print colored('               Runtime: %f [s] (%s)' % (runtime_s, str(datetime.timedelta(seconds=runtime_s))), 'yellow')
            print colored('  Number of bytes used: %i' % byte_amount, 'yellow')
            print colored('Number of objects used: %i' % object_amount, 'yellow')
            print colored(' Estimated flight time: %.02f [s] (%s)' % (estimated_flight_time, str(datetime.timedelta(seconds=estimated_flight_time))), 'yellow') # TODO
            print colored('   Number of waypoints: %i' % (no_waypoints), 'yellow')
            print colored('                  Path: %s' % (str(path_planned)), 'yellow')

        # Save statistics to file
        if SAVE_STATISTICS_TO_FILE:
            now = datetime.datetime.now()
            file_name = ('PP_%d-%02d-%02d-%02d-%02d.csv' % (now.year, now.month, now.day, now.hour, now.minute))
            sub_folder = 'results'
            file_name = sub_folder+'/'+file_name
            output_file_CSV = open(file_name, 'w')
            output_writer_CSV = csv.writer(output_file_CSV,quoting=csv.QUOTE_MINIMAL)
            fields = ['path planner', 'path fitness [unitless]', 'total distance [m]', 'horizontal distance [m]', 'vertical distance [m]', 'runtime [s]', 'bytes used', 'objects used', 'flight time [s]', 'waypoints', 'start point', 'goal point', 'path']
            output_writer_CSV.writerow(fields)
            data = [used_path_planner, path_planned_fitness, total_dist, horz_dist, vert_dist, runtime_s, byte_amount, object_amount, estimated_flight_time, no_waypoints, start_point_3dDICT, goal_point_3dDICT, path_planned]
            output_writer_CSV.writerow(data)
            output_file_CSV.close()
