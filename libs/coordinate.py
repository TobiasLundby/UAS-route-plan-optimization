#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: coordinate transform class which handles coordinates in geodetic, UTM, and pseudo mercator format
    License: BSD 3-Clause
"""

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

""" Import libraries """
from pyproj import Proj, transform # Transform coordiantes between projections
from transverse_mercator_py.utm import utmconv # Convert between geodetic and UTM coordinates

class coordinate_transform():
    def __init__(self, debug = False, lock_UTM_zone = None):
        """
        Init method
        Input: optional debug parameter which toogles debug messages
        Output: none
        """
        self.debug = debug

        self.geodetic_proj  = Proj('+init=EPSG:4326')  # EPSG:4326 - WGS 84 / World Geodetic System 1984, used in GPS - https://epsg.io/4326
        self.OSM_proj       = Proj('+init=EPSG:3857')  # EPSG:3857 - WGS 84 / Pseudo-Mercator - https://epsg.io/3857 - OSM projection

        # Instantiate utmconv class
        self.uc = utmconv()
        self.first_UTM_conv = True
        if lock_UTM_zone != None:
            self.uc.set_zone_override(lock_UTM_zone)

    def Geodetic2PseudoMercator(self, lat, lon = False):
        """
        Converts geodetic latitude and longitude to pseudo mercator (OSM) x and y
        Input: latitude, longitude (EPSG:4326) or array of lat and lon as the lat input
        Output: x, y(EPSG:3857)
        """
        if lon == False and len(lat) == 2:
            x,y = transform(self.geodetic_proj, self.OSM_proj, lat[1], lat[0])
        else:
            x,y = transform(self.geodetic_proj, self.OSM_proj, lon, lat)
        return (x,y)
    def PseudoMercator2Geodetic(self, x,y):
        """
        Converts x and y pseudo mercator (OSM) to geodetic latitude and longitude
        Input: x, y(EPSG:3857)
        Output: latitude, longitude (EPSG:4326)
        """
        lon, lat = transform(self.OSM_proj, self.geodetic_proj, x, y)
        return (lat, lon)

    def UTM2geodetic(self, hemisphere, zone, y, x):
        return self.uc.utm_to_geodetic(hemisphere, zone, y, x)
    def geodetic2UTM(self, lat, lon):
        """
        Converts geodetic coordinates to UTM coordiantes
        Note that it locks the zone after the first conversion
        Input: latitude and longitude [dd]
        Output: hemisphere, zone, zone letter, easting, northing
        """
        if self.first_UTM_conv:
            (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(lat, lon)
            self.uc.set_zone_override(zone)
            print 'zone override to', zone
            self.first_UTM_conv = False
            return (hemisphere, zone, letter, easting, northing)
        return self.uc.geodetic_to_utm(lat, lon)

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

    def pos3d_geodetic2pos3d_UTM_multiple(self, pos3d_geodetic_multiple):
        """ Converts multiple geodetic pos3d to UTM pos3d
        Input: multiple geodetic points as 3 value 1 array of lat, lon, alt_rel
        Output: multiple UTM points as 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        """
        converted_points = []
        for element in pos3d_geodetic_multiple:
            converted_points.append( self.pos3d_geodetic2pos3d_UTM(element) )
        return converted_points
    def pos3d_UTM2pos3d_geodetic_multiple(self, pos3d_UTM_multiple):
        """ Converts multiple UTM pos3d to geodetic pos3d
        Input: multiple UTM points as 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        Output: multiple geodetic points as 3 value 1 array of lat, lon, alt_rel
        """
        converted_points = []
        for element in pos3d_UTM_multiple:
            converted_points.append( self.pos3d_UTM2pos3d_geodetic(element) )
        return converted_points
    def pos3d_geodetic2pos3d_UTM(self, pos3d_geodetic):
        """ Converts geodetic pos3d to UTM pos3d
        Input: 3 value 1 array of lat, lon, alt_rel
        Output: 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        """
        if self.first_UTM_conv:
            (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3d_geodetic[0],pos3d_geodetic[1])
            self.uc.set_zone_override(zone)
            print 'zone override to', zone
            self.first_UTM_conv = False
            return [easting, northing, pos3d_geodetic[2], hemisphere, zone, letter]
        (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3d_geodetic[0],pos3d_geodetic[1])
        return [easting, northing, pos3d_geodetic[2], hemisphere, zone, letter]


        # (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3d_geodetic[0],pos3d_geodetic[1])
        # return [easting, northing, pos3d_geodetic[2], hemisphere, zone, letter]
    def pos3d_UTM2pos3d_geodetic(self, pos3d_UTM):
        """ Converts UTM pos3d to geodetic pos3d
        Input: 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        Output: 3 value 1 array of lat, lon, alt_rel
        """
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic (pos3d_UTM[3], pos3d_UTM[4], pos3d_UTM[0], pos3d_UTM[1])
        return [back_conv_lat, back_conv_lon, pos3d_UTM[2]]
    def pos3d_UTM2pos3d_geodetic_special(self, pos3d_UTM):
        """ Converts UTM pos3d to geodetic pos3d
        Input: 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        Output: 3 value 1 array of lat, lon, alt_rel
        """
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic ('N', 32, pos3d_UTM[0], pos3d_UTM[1])
        return [back_conv_lat, back_conv_lon, pos3d_UTM[2]]

    def generate_pos3d_UTM_list(self, easting, northing, z_rel, hemisphere = 'N', zone=32, letter='U'):
        """ Generates a 3d UTM point as list
        Input: easting, northing, z_rel and optional hemisphere, zone, and letter
        Output: 6 value 1 element array of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        """
        return [float(easting), float(northing), float(z_rel), hemisphere, zone, letter]
    def generate_pos3d_UTM_dict(self, easting, northing, z_rel, hemisphere = 'N', zone=32, letter='U'):
        """ Generates a 3d UTM point as list
        Input: easting, northing, z_rel and optional hemisphere, zone, and letter
        Output: 6 value 1 element dict of y/easting, x/norting, alt_rel, hemisphere, zone, and letter
        """
        return {'y':float(easting), 'x': float(northing), 'z_rel':float(z_rel), 'hemisphere': hemisphere, 'zone': zone, 'letter': letter}
    def generate_pos4d_UTM_list(self, easting, northing, z_rel, time_rel = 0.0, hemisphere = 'N', zone=32, letter='U'):
        return [float(easting), float(northing), float(z_rel), float(time_rel), hemisphere, zone, letter]
    def generate_pos4d_UTM_dict(self, easting, northing, z_rel, time_rel = 0.0, hemisphere = 'N', zone=32, letter='U'):
        return {'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':float(easting),'x':float(northing),'z_rel':float(z_rel),'time_rel':float(time_rel)}

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
        if self.first_UTM_conv:
            (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3dDICT['lat'],pos3dDICT['lon'])
            self.uc.set_zone_override(zone)
            print 'zone override to', zone
            self.first_UTM_conv = False
            return {'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':easting,'x':northing,'z_rel':pos3dDICT['alt_rel'],'time_rel':None}
        (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3dDICT['lat'],pos3dDICT['lon'])
        return {'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':easting,'x':northing,'z_rel':pos3dDICT['alt_rel'],'time_rel':None}

        # (hemisphere, zone, letter, easting, northing) = self.uc.geodetic_to_utm(pos3dDICT['lat'],pos3dDICT['lon'])
        # return {'hemisphere':hemisphere,'zone':zone,'letter':letter,'y':easting,'x':northing,'z_rel':pos3dDICT['alt_rel'],'time_rel':None}
    def pos4dDICT_UTM2pos4dDICT_geodetic(self, pos4dDICT_UTM):
        """ Converts pos4dDICT (UTM) to pos4dDICT (geodetic)
        Input: 7 value 1 element DICT of hemisphere, zone, letter, y, x, z_rel, time_rel
        Output: 4 value 1 element DICT of lat, lon, alt_rel (relative to start height), time_rel
        """
        (back_conv_lat, back_conv_lon) = self.uc.utm_to_geodetic (pos4dDICT_UTM['hemisphere'], pos4dDICT_UTM['zone'], pos4dDICT_UTM['y'], pos4dDICT_UTM['x'])
        return {'lat':round(back_conv_lat,5),'lon':round(back_conv_lon,5),'alt_rel':round(pos4dDICT_UTM['z_rel'],1),'time_rel':round(pos4dDICT_UTM['time_rel'],2)}

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

    def utm_test(self, test_lat, test_lon):
        """
        Test of the UTM class which uses 2 geodetic points and determines the error from forward and back conversion
           Heavily inspired by https://github.com/FroboLab/frobomind/blob/master/fmLib/math/geographics/transverse_mercator/src/transverse_mercator_py/utm_test.py
        Input: 2 geodetic points
        Output: None but output in terminal
        """
        # Print initial geodetic points
        print '\nTest position [deg]:'
        print ' latitude:  %.10f'  % (test_lat)
        print 'longitude:  %.10f'  % (test_lon)

        # Convert from geodetic to UTM
        (hemisphere, zone, letter, easting, northing) = self.coord_conv.geodetic2UTM (test_lat,test_lon)
        print 'Converted from geodetic to UTM [m]:'
        print '%d %c %.5fe %.5fn' % (zone, letter, easting, northing)

        # Convert back from UTM to geodetic
        (back_conv_lat, back_conv_lon) = self.coord_conv.UTM2geodetic (hemisphere, zone, easting, northing)
        print 'Converted back from UTM to geodetic [deg]:'
        print ' latitude:  %.10f'  % (back_conv_lat)
        print 'longitude:  %.10f'  % (back_conv_lon)

        # Determine conversion position error [m]
        lat_err = abs(back_conv_lat-test_lat)
        lon_err = abs(back_conv_lon-test_lon)
        earth_radius = 6378137.0 # [m]
        lat_pos_err = lat_err/360.0 * 2*pi*earth_radius
        lon_pos_err = lon_err/360.0 * 2*pi*(cos(back_conv_lat)*earth_radius)
        print 'Positional error from the two conversions [m]:'
        print ' latitude:  %.09f'  % (lat_pos_err)
        print 'longitude:  %.09f'  % (lon_pos_err)
