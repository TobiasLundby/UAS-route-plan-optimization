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
from droneid_simulator import droneid_simulator

from urllib2 import urlopen, URLError, HTTPError
import csv
import json
import time
import datetime # datetime.now
import pytz # timezones in the datetime format
#print 'Import done\n'

forever = 60*60*24*365*100 # 100 years excl. leap year
internet_connection_tries = 5

class droneid_data():
    def __init__(self, debug):
        self.internet_tester = internet_tools()
        self.url = 'https://droneid.dk/tobias/droneid.php'
        self.debug = debug
        self.aircraft_count = 0
        self.DroneIDdataFields = ['time_stamp','time_since_epoch','id','name','lat','lon','alt','acc','fix','lnk','eng','sim']
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
    def download_data(self):
        self.drone_count = 0
        self.DroneIDdataRaw = []
        self.DroneIDdataStructured = []
        if self.debug:
            print 'Attempting to download'
        try:
            response = urlopen(self.url)
        except HTTPError as e:
            result = '%s' % (e.code)
        except URLError as e:
            result = '%s %s' % (e.code, e.reason)
        else:
            if self.debug:
                print 'No errors encountered during download, attempting to read result'
        itr = 0
        for line in response:

            line = line.rstrip()
            #print line
            if line != "":
                itr = itr+1
                #print "Line",itr,":",line
                self.DroneIDdataRaw.append(line)

                csv_reader = csv.reader( [ line ] )
                for row in csv_reader:
                    row[1] = int(row[1]) # epoch time
                    row[2] = int(row[2]) # id
                    row[4] = float(row[4]) # lat
                    row[5] = float(row[5]) # lng
                    row[6] = int(row[6]) # alt
                    row[7] = int(row[7]) # acc
                    row[8] = int(row[8]) # fix
                    if row[9] == '': # catch a simulated drone and set link to 100%
                        row[9] = int(100)
                    else:
                        row[9] = int(row[9]) # lnk
                    if row[10] == '': # catch a simulated drone and set battery to 100%
                        row[10] = int(100) # eng
                    else:
                        row[10] = int(row[10])
                    row[11] = int(row[11]) # sim
                    self.DroneIDdataStructured.append(row)
        #print itr
        self.drone_count = itr
        if self.drone_count == 0:
            print "No DroneIDs seems to be online"
        if self.debug:
            print "Entries:",self.drone_count
        response.close()
        if self.debug:
            print "Result read successfully"

        # Make JSON Data
        # for row in self.DroneIDdataStructured:
        #     print row
    def update_data(self):
        print '\nUpdate begun'
        self.drone_count_new = 0

        if self.debug:
            print 'Attempting to download'
        try:
            response = urlopen(self.url)
        except HTTPError as e:
            result = '%s' % (e.code)
        except URLError as e:
            result = '%s %s' % (e.code, e.reason)
        else:
            if self.debug:
                print 'No errors encountered during download, attempting to read result'

        # begin data analysis
        itr = 0
        for line in response:
            line = line.rstrip()
            #print line
            if line != "":
                itr = itr+1 # note +1 higher than the index
                #print "Line",itr,":",line
                csv_reader = csv.reader( [ line ] )
                for row in csv_reader:
                    row[1] = int(row[1]) # epoch time
                    row[2] = int(row[2]) # id
                    row[4] = float(row[4]) # lat
                    row[5] = float(row[5]) # lng
                    row[6] = int(row[6]) # alt
                    row[7] = int(row[7]) # acc
                    row[8] = int(row[8]) # fix
                    if row[9] == '': # catch a simulated drone and set link to 100%
                        row[9] = int(100)
                    else:
                        row[9] = int(row[9]) # lnk
                    if row[10] == '': # catch a simulated drone and set battery to 100%
                        row[10] = int(100) # eng
                    else:
                        row[10] = int(row[10])

                    drone_index_old = self.get_drone_index_from_id(row[2])
                    if drone_index_old != None:
                        print "Already seen", row[3], ", id:", drone_index_old
                        # Update values: time, epoch, lat, lng, alt, acc, fix, lng, eng
                        self.DroneIDdataStructured[drone_index_old][0] = row[0]
                        self.DroneIDdataStructured[drone_index_old][1] = row[1]
                        self.DroneIDdataStructured[drone_index_old][4] = row[4]
                        self.DroneIDdataStructured[drone_index_old][5] = row[5]
                        self.DroneIDdataStructured[drone_index_old][6] = row[6]
                        self.DroneIDdataStructured[drone_index_old][7] = row[7]
                        self.DroneIDdataStructured[drone_index_old][8] = row[8]
                        self.DroneIDdataStructured[drone_index_old][9] = row[9]
                        self.DroneIDdataStructured[drone_index_old][10] = row[10]

                        # Update the raw entry
                        self.DroneIDdataRaw[drone_index_old] = line
                    else:
                        print "New", row[3], ", appending specific drone data"
                        row[11] = int(row[11]) # sim, not used for updating
                        # Add entry to structured
                        self.DroneIDdataRaw.append(line)
                        self.DroneIDdataStructured.append(row)
                        self.drone_count = self.drone_count + 1

        print 'Update done\n'
    def print_data(self):
        if self.drone_count > 0:
            print self.DroneIDdataStructured
    def print_raw(self):
        if self.drone_count > 0:
            print self.DroneIDdataRaw
    def print_CSV(self):
        if self.drone_count > 0:
            print ",".join(self.DroneIDdataFields)
            for line in self.DroneIDdataRaw:
                print line
    def print_drone_pretty(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            if self.get_sim(drone_index):
                print "NOTE: SIMULATED DRONE (link quality and battery SoC hardcoded)"
            print "ID:          ", self.get_id(drone_index)
            print "Name:        ", self.get_name(drone_index)
            print "Timestamp:   ", self.get_time_stamp(drone_index)
            print "Epoch time:  ", self.get_time_since_epoch(drone_index),'- local:', datetime.datetime.fromtimestamp(self.get_time_since_epoch(drone_index)).strftime('%Y-%m-%d %H:%M:%S'),'- UTC:',datetime.datetime.fromtimestamp(self.get_time_since_epoch(drone_index), pytz.UTC).strftime('%Y-%m-%d %H:%M:%S')
            print "Lat:         ", self.get_lat(drone_index)
            print "Lng:         ", self.get_lng(drone_index)
            print "OSM link:    ", 'http://www.openstreetmap.org/?mlat=%s&mlon=%s&zoom=16' % (self.get_lat(drone_index),self.get_lng(drone_index))
            print "Google link:  "  'http://maps.google.com/maps?q=%s+%s' % (self.get_lat(drone_index),self.get_lng(drone_index))
            print "Altitude:    ", self.get_alt_m(drone_index), 'm'
            print "Accuracy:    ", self.get_acc(drone_index)
            print "Fix type:    ", self.get_fix(drone_index)
            print "Link quailty:", self.get_lnk(drone_index), '%'
            print "Battery SoC: ", self.get_eng(drone_index), '%'
            print "Simulated:   ", self.get_sim(drone_index)
    def print_format(self):
        print "time_stamp,time_since_epoch,id,name,lat,lon,alt,acc,fix,lnk,eng,sim"
    def print_description(self):
        print """\nField description:
        time_stamp: Formatted time stamp
        time: seconds since Epoch
        id:droneid
        name: drone name (if any, if not it should be represented by the id)
        lat: [Decimal degrees]
        lon: [Decimal degrees]
        alt: [m] zero if fix is FIX_CELL
        acc: Horizontal position accuracy
        fix: Position fix
        	$FIX_NO_FIX = 0; // no position available
        	$FIX_SPS = 1; // standard positioning service
        	$FIX_DGPS = 2; // differential GPS
        	$FIX_RTK_FIXED = 4; // real time kinematics - fixed solution
        	$FIX_RTK_FLOAT = 5; // real time kinematics - float solution
        	$FIX_CELL = 6; // position of nearby cell phone tower
        lnk: GSM link signal quality 0-100%
        eng: Energy (battery) level: 0-100%
        sim: True if simulated data """
    def get_time_stamp(self, drone_index):
        # GMT+1
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][0]
        else: return None
    def get_time_since_epoch(self, drone_index):
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][1]
        else: return None
    def get_time_since_epoch_formatted_UTC(self, drone_index):
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return datetime.datetime.fromtimestamp(self.DroneIDdataStructured[drone_index][1], pytz.UTC).strftime('%Y-%m-%d %H:%M:%S')
        else: return None
    def get_time_since_epoch_formatted_local(self, drone_index):
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return datetime.datetime.fromtimestamp(self.DroneIDdataStructured[drone_index][1]).strftime('%Y-%m-%d %H:%M:%S')
        else: return None
    def get_id(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][2]
        else: return None
    def get_name(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][3]
        else: return None
    def get_lat(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][4]
        else: return None
    def get_lng(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][5]
        else: return None
    def get_alt_m(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][6]
        else: return None
    def get_alt_ft(self, drone_index):
        # unit: feet
        return self.get_alt_m(drone_index)*3.28084
    def get_acc(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][7]
        else: return None
    def get_fix(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][8]
        else: return None
    def get_lnk(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][9]
        else: return None
    def get_eng(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][10]
        else: return None
    def get_sim(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][11]
        else: return None
    def get_drone_data(self, drone_index):
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index]
        else: return None
    def get_drone_data_from_name(self, drone):
        if self.drone_count > 0:
            if drone != "" and type(drone)==str:
                for line in self.DroneIDdataStructured:
                    if line[3] == drone:
                        return line
                return []
            else: return None
        else: return None
    def get_drone_index_from_name(self, drone):
        if self.drone_count > 0:
            if drone != "" and type(drone)==str:
                itr = 0
                for line in self.DroneIDdataStructured:
                    if line[3] == drone:
                        return itr
                    itr = itr+1
                return None
            else: return None
        else: return None
    def get_drone_index_from_id(self, drone_id):
        if self.drone_count > 0:
            if drone_id >= 0 and type(drone_id)==int:
                itr = 0
                for line in self.DroneIDdataStructured:
                    if line[2] == drone_id:
                        return itr
                    itr = itr+1
                return None
            else: return None
        else: return None

if __name__ == '__main__':
    # Run self test
    # self test function not made yet

    LOG_TEST_DATA = [
		{
			'aid': 900,
			'lat': 55.395,
			'lng': 10.371,
			'alt': 50,
		},
		{
			'aid': 901,
			'lat': 55.396,
			'lng': 10.372,
			'alt': 100,
		}
	]

    simulator_module = droneid_simulator()
    simulator_module.send_log_entry (LOG_TEST_DATA[0])

    droneid_module = droneid_data(False)
    droneid_module.download_data()
    #droneid_module.print_data()
    #droneid_module.print_drone_pretty(0)
    droneid_module.print_raw()
    droneid_module.print_data()

    time.sleep(2)
    simulator_module.send_log_entry (LOG_TEST_DATA[0])
    simulator_module.send_log_entry (LOG_TEST_DATA[1])
    droneid_module.update_data()
    droneid_module.print_raw()
    droneid_module.print_data()

    time.sleep(2)
    simulator_module.send_log_entry (LOG_TEST_DATA[0])
    simulator_module.send_log_entry (LOG_TEST_DATA[1])
    droneid_module.update_data()
    droneid_module.print_raw()
    droneid_module.print_data()
    #print droneid_module.get_drone_data(0)
    #print droneid_module.get_drone_data_from_name('000901')
    #print droneid_module.get_drone_index_from_id(901)
    #print droneid_module.get_drone_index_from_name('000901')
    #print droneid_module.get_time_since_epoch_formatted_local(0)
    #droneid_module.print_description()
    #droneid_module.print_raw()
    #droneid_module.print_CSV()
