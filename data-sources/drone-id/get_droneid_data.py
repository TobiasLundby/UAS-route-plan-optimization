#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-03-09 TL First version
"""

"""
Description:
This scripts downloads DroneID data from a source provided by Kjeld Jensen (kjen@mmmi.sdu.dk) and parses it accordingly for easy use
The script support redownload using the 'download_data' method
License: BSD 3-Clause
Data source: https://droneid.dk/tobias/droneid.php
Data readme: https://droneid.dk/tobias/vejledning.txt
"""

#print 'Importing libraries'
import os, sys # exit (use quit() when you only want to terminate spawning script, not all)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from internet_tools import internet_tools

from urllib2 import urlopen, URLError, HTTPError
import csv
import json
import time
import datetime # datetime.now
#print 'Import done\n'

forever = 60*60*24*365*100
internet_connection_tries = 5

class droneid_data():
    def __init__(self, debug):
        self.internet_tester = internet_tools()
        self.debug = debug
        self.aircraft_count = 0
        self.DroneIDdataFields = ['time_stamp','time_since_epoch','icao','flight','lat','lon','alt','track','speed']
        self.DroneIDdataRaw = []
        self.DroneIDdataStructured = []
        for x in range(1, internet_connection_tries+1):
            if x != 1:
                print "Checking internet connection, try", x
            if self.internet_tester.internet_on(2) == False:
                if x == internet_connection_tries:
                    print "No internet connection, terminating\n"
                    sys.exit()
            else:
                break

if __name__ == '__main__':
    # Run self test
    # self test function not made yet

    droneid_module = droneid_data(False)
