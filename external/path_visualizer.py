#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-04-19 TL Started class
2018-04-20 TL Class completed
"""

"""
Description:
Class for sending paths for 2D and 3D visualization on http://uas.heltner.net/routes
Visualization website source: https://github.com/UAS-Bachelor/uas-tracking
License: BSD 3-Clause
"""

#print 'Importing libraries'
import requests
import sys
from termcolor import colored
#print 'Import done\n'

""" User defines """
DEFAULT_AID = 900

class path_visualizer():
    def __init__(self, debug = False):
        self.debug = debug
    def generate_JSON_path_from_raw_4val_arr(self, path, aid):
        """
        Input: 4 value array of lat, lon, alt, time, and AID; if AID is not provided it will be defaulted
        Output: Path in JSON format required by visualizer framework
        """
        if aid == None:
            aid = DEFAULT_AID
            if self.debug:
                print colored('Trying to generate JSON path with default AID (%d)' % DEFAULT_AID, 'yellow')
        path_JSON = []
        if len(path[0]) == 4:
            for element in path:
                path_JSON.append({'aid': aid,'lat': element[0],'lon': element[1],'alt': element[2],'time': element[3]})
        if path_JSON == []:
            print colored('Could not generate JSON path', 'red')
            sys.exit(1)
        else:
            print colored('JSON path generated', 'green')
        return path_JSON
    def generate_JSON_path_from_raw_4val_dict(self, path):
        """
        Input: 4 value DICT (array lat, lon, alt, time) and AID; if AID is not provided it will be defaulted
        Output: Path in JSON format required by visualizer framework
        """
        if self.debug:
            print colored('No AID specified so defaulted to %d' % DEFAULT_AID, 'yellow')
        path_JSON = []
        for element in path:
            path_JSON.append({'aid': DEFAULT_AID,'lat': element['lat'],'lon': element['lon'],'alt': element['alt'],'time': element['time']})
        return path_JSON
    def generate_JSON_path_from_raw_5val_dict(self, path):
        """
        Input: 5 value DICT (array of AID, lat, lon, alt, time)
        Output: Path in JSON format required by visualizer framework
        """
        path_JSON = []
        for element in path:
            path_JSON.append({'aid': element['aid'],'lat': element['lat'],'lon': element['lon'],'alt': element['alt'],'time': element['time']})
        return path_JSON
    def visualize_path(self, path, aid=False):
        """
        Input:
            1. 5 value DICT (array of AID, lat, lon, alt, time)
            2. 4 value DICT (array of lat, lon, alt, time) and AID; if AID is not provided it will be defaulted
            3. 4 value array of lat, lon, alt, time, and AID; if AID is not provided it will be defaulted
        Output: route id (int) of path on http://uas.heltner.net/routes
        """
        # If aid is not specified the data must be given as DICT with aid embedded. If aid is specified the path needs to be a simple 4 element array og lat, lon, alt, time
        # Check if there is any data
        if isinstance(path, list) and len(path) > 0:
            path_JSON = []
            # Checking format and generate JSON path
            if aid == False:
                try:
                    tmp_var = path[0]['lat']
                except TypeError:
                    print colored('Input not as expected; it seems as the path provided is a 4 value array containing lat, lon, alt, time but missing aid', 'red')
                    path_JSON = self.generate_JSON_path_from_raw_4val_arr(path, None)
                else:
                    try:
                        tmp_var = path[0]['aid']
                    except KeyError:
                        path_JSON = self.generate_JSON_path_from_raw_4val_dict(path)
                    else:
                        path_JSON = self.generate_JSON_path_from_raw_5val_dict(path)
            else:
                path_JSON = self.generate_JSON_path_from_raw_4val_arr(path, aid)

            # Print path
            if self.debug:
                print path_JSON

            # Send the path to http://uas.heltner.net/routes
            try:
                r = requests.post("http://uas.heltner.net/routes", json=path_JSON, timeout=2)
            except requests.exceptions.Timeout:
                # Maybe set up for a retry, or continue in a retry loop
                print colored('Request has timed out', 'red')
            except requests.exceptions.TooManyRedirects:
                # Tell the user their URL was bad and try a different one
                print colored('Request has too many redirects', 'red')
            except requests.exceptions.RequestException as e:
                # catastrophic error. bail.
                print colored('Request error', 'red')
                print colored(e, 'yellow')
                # sys.exit(1)
            else:
                #print(r.status_code, r.reason)
                #print(r.text)
                if r.status_code == 200 and r.reason == 'OK':
                    route_id = int(r.text.strip())
                    if self.debug:
                        print colored('Path uploaded, route ID %s: %s, %s' % (route_id, 'http://uas.heltner.net/routes/'+str(route_id)+'/2d', 'http://uas.heltner.net/routes/'+str(route_id)+'/3d'), 'green')
                    return route_id
                else:
                    print colored('Path NOT uploaded', 'red')
        return None
    def delete_route(self, routeid):
        """
        Input: route id to be deleted on http://uas.heltner.net/routes
        Output: deleted route id (the route id returned by the delete request)
        """
        if isinstance(routeid, (int, long)):
            try:
                r = requests.delete("http://uas.heltner.net/routes", json={'routeid': routeid}, timeout=2)
            except requests.exceptions.Timeout:
                # Maybe set up for a retry, or continue in a retry loop
                print colored('Request has timed out', 'red')
            except requests.exceptions.TooManyRedirects:
                # Tell the user their URL was bad and try a different one
                print colored('Request has too many redirects', 'red')
            except requests.exceptions.RequestException as e:
                # catastrophic error. bail.
                print colored('Request error', 'red')
                print colored(e, 'yellow')
                # sys.exit(1)
            else:
                #print(r.status_code, r.reason)
                #print(r.text)
                if r.status_code == 200 and r.reason == 'OK':
                    route_id_response = int(r.text.strip())
                    if self.debug:
                        print colored('Path/route deleted, route ID %d (response ID %d)' % (routeid, route_id_response), 'green')
                    return route_id_response
                else:
                    print colored('Path/route NOT deleted', 'red')
        return None


if __name__ == '__main__':
    LOG_TEST_DATA_wAID = [
        {
            'aid': 900,
            'lat': 55.399,
            'lon': 10.385,
            'alt': 50,
            'time': 1524162422
        },
        {
            'aid': 900,
            'lat': 55.400,
            'lon': 10.386,
            'alt': 100,
            'time': 1524162482
        }
    ]
    LOG_TEST_DATA_woAID = [
            {
            'lat': 55.399,
            'lon': 10.385,
            'alt': 50,
            'time': 1524162422
        },
        {
            'lat': 55.400,
            'lon': 10.386,
            'alt': 100,
            'time': 1524162482
        }
    ]
    TEST_DATA = [
        [55.399, 10.385, 50, 1524162422],
        [55.400, 10.386, 100, 1524162482]
    ]

    path_visualizer_module = path_visualizer(True)
    route_id = path_visualizer_module.visualize_path(LOG_TEST_DATA_wAID)
    route_id = path_visualizer_module.visualize_path(LOG_TEST_DATA_woAID)
    route_id = path_visualizer_module.visualize_path(TEST_DATA)
    route_id = path_visualizer_module.visualize_path(TEST_DATA,900)
    print path_visualizer_module.delete_route(route_id)
