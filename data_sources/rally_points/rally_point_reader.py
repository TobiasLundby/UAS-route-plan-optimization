#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description:
Reader for the rally points stored in a CSV file
License: BSD 3-Clause
Data readme: https://droneid.dk/tobias/vejledning.txt
"""

import csv

class rally_point_reader():
    def __init__(self, input_filename, debug = False):
        self.debug = debug
        self.filename = input_filename

        self.rally_points_dict = []
        self.rally_points_list = []

    def parse_rally_points(self):
        with open(self.filename) as csvfile:
            reader = csv.DictReader(csvfile, delimiter=',')
            for row in reader:
                name = str(row['name'])
                lat = float(row['latitude_dd'])
                lon = float(row['longitude_dd'])
                z_rel = float(row['z_rel_m'])
                if self.debug:
                    print 'Rally point \'%s\' with latitude %.05f and longitude %.05f' % (name, lat, lon)
                self.rally_points_dict.append({'lat': lat, 'lon': lon, 'z_rel': z_rel, 'name': name})
                self.rally_points_list.append([ lat, lon, z_rel, name ])

    def get_rally_points_dict(self):
        return self.rally_points_dict
    def get_rally_points_list(self):
        return self.rally_points_list

if __name__ == "__main__":
    rally_point_module = rally_point_reader('rally_points.csv', debug = True)
    rally_point_module.parse_rally_points()
    print rally_point_module.get_rally_points_list()
