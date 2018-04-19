#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-04-19 TL Started class
"""

"""
Description:
Class for sending paths for 2D and 3D visualization on http://uas.heltner.net/routes
Visualization website source: https://github.com/UAS-Bachelor/uas-tracking
License: BSD 3-Clause
"""

#print 'Importing libraries'
import requests
from termcolor import colored
#print 'Import done\n'

class path_visualizer():
    def __init__(self, debug = False):
        self.debug = debug
    def visualize_path(self, path, aid=False):
        # If aid is not specified the data must be given as DICT with aid embedded. If aid is specified the path needs to be a simple 4 element array og lat, lon, alt, time
        # Check if there is any data
        if  isinstance(path, list) and len(path) > 0:
            path_JSON = []
            # some code for generating the right format
            if aid == False:
                # code
                for element in path:
                    path_JSON.append({'aid': element['aid'],'lat': element['lat'],'lon': element['lng'],'alt': element['alt'],'time': element['time']})
                print path_JSON
            else:
                # code
                arr = []
            # send the path to http://uas.heltner.net/routes
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
                    if self.debug:
                        print colored('Path uploaded, route ID %s: %s, %s' % (r.text.strip(), 'http://uas.heltner.net/routes/'+r.text.strip()+'/2d', 'http://uas.heltner.net/routes/'+r.text.strip()+'/3d'), 'green')
                else:
                    print colored('Path NOT uploaded', 'red')


if __name__ == '__main__':
    LOG_TEST_DATA_wAID = [
        {
            'aid': 900,
            'lat': 55.399,
            'lng': 10.385,
            'alt': 50,
            'time': 1524162422
        },
        {
            'aid': 900,
            'lat': 55.400,
            'lng': 10.386,
            'alt': 100,
            'time': 1524162482
        }
    ]
    LOG_TEST_DATA_woAID = [
            {
            'lat': 55.399,
            'lng': 10.385,
            'alt': 50,
            'time': 1524162422
        },
        {
            'lat': 55.400,
            'lng': 10.386,
            'alt': 100,
            'time': 1524162482
        }
    ]

    path_visualizer_module = path_visualizer(True)
    path_visualizer_module.visualize_path(LOG_TEST_DATA_wAID)


    arr = []
    arr.append({'aid': 900,'lat': 55.400,'lon': 10.385,'alt': 50})
    print LOG_TEST_DATA_wAID[1]['aid']
    print arr
