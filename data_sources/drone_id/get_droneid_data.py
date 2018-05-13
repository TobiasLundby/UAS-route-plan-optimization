#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-03-09 TL First version
2018-03-12 TL Started expanding functions
2018-03-14 TL Functions improved and added update function
2018-03-17 TL Stability improvements and testing
2018-03-18 TL Continued 'stability improvements ...'
2018-03-19 TL Continued 'stability improvements ...'
"""

"""
Description:
This scripts downloads DroneID data from a source provided by Kjeld Jensen (kjen@mmmi.sdu.dk) and parses it accordingly for easy use
The script support redownload using the 'download_data' method.
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
inf = 4294967295 # 32bit from 0
internet_connection_tries = 5

run_modes = ['test']

class droneid_data():
    def __init__(self, debug = False, force_sim_to_real = False):
        self.internet_tester = internet_tools()
        self.url = 'https://droneid.dk/tobias/droneid.php' # API URL
        self.debug = debug
        self.force_sim_to_real = force_sim_to_real
        self.aircraft_count = 0
        self.DroneIDdataFieldsRaw = ['time_stamp','time_since_epoch','id','name','lat','lon','alt','acc','fix','lnk','eng','sim']
        self.DroneIDdataFields = ['time_stamp','time_since_epoch','id','name','lat','lon','alt','acc','fix','lnk','eng','sim','time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS']
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
        """
        Downloads the active drones from droneID
        Input: none
        Output: bool (True = download successful, False = download failed)
        """
        self.drone_count = 0
        self.DroneIDdataRaw = []
        self.DroneIDdataStructured = []

        if self.debug:
            print 'Attempting to download'

        try:
            response = urlopen(self.url)
        except HTTPError, e:
            print 'The server couldn\'t fulfill the request.'
            print 'Error code: ', e.code
            return False
        except URLError, e:
            print 'Failed to reach server.'
            print 'Reason: ', e.reason
            return False
        except IOError, e:
            if hasattr(e, 'reason'):
                print 'Failed to reach server.'
                print 'Reason: ', e.reason
            elif hasattr(e, 'code'):
                print 'The server couldn\'t fulfill the request.'
                print 'Error code: ', e.code
            return False
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
                        row[6] = float(row[6]) # alt
                        row[7] = float(row[7]) # acc
                        row[8] = int(row[8]) # fix
                        if row[9] == '': # catch a simulated drone and set link to 100%
                            row[9] = int(100)
                        else:
                            row[9] = int(row[9]) # lnk
                        if row[10] == '': # catch a simulated drone and set battery to 100%
                            row[10] = int(100) # eng
                        else:
                            row[10] = int(row[10])
                        if self.force_sim_to_real == True: row[11] = 0
                        else: row[11] = int(row[11]) # sim
                        # Make arrays for 'time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS'
                        row.append([])
                        row.append([])
                        row.append([])
                        row.append([])
                        #row[12].append(row[1])
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
            return True
    def update_data(self, max_history_data = inf):
        """
        Updates the internal class drone data by redownloading and adding/updating
        Input: optional limit on historical data
        Output: bool (True = download successful, False = download failed)
        """
        if self.debug:
            print '\nUpdate begun'
        self.drone_new_data_count = 0

        if self.debug:
            print 'Attempting to download'

        try:
            response = urlopen(self.url)
        except HTTPError, e:
            print 'The server couldn\'t fulfill the request.'
            print 'Error code: ', e.code
            return False
        except URLError, e:
            print 'Failed to reach server.'
            print 'Reason: ', e.reason
            return False
        except IOError, e:
            if hasattr(e, 'reason'):
                print 'Failed to reach server.'
                print 'Reason: ', e.reason
            elif hasattr(e, 'code'):
                print 'The server couldn\'t fulfill the request.'
                print 'Error code: ', e.code
            return False
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
                        row[6] = float(row[6]) # alt
                        row[7] = float(row[7]) # acc
                        row[8] = int(row[8]) # fix
                        if row[9] == '': # catch a simulated drone and set link to 100%
                            row[9] = int(100)
                        else:
                            row[9] = int(row[9]) # lnk
                        if row[10] == '': # catch a simulated drone and set battery to 100%
                            row[10] = int(100) # eng
                        else:
                            row[10] = int(row[10])

                        drone_index_old = self.get_drone_index_from_id(row[2]) # match id's to see if it exists
                        if drone_index_old != None:
                            if self.debug:
                                print "Already seen", row[3], ", id:", drone_index_old
                            if self.DroneIDdataStructured[drone_index_old][1] < row[1] or self.DroneIDdataStructured[drone_index_old][7] < row[7]:
                                # New or better data received
                                # Remove data which exceeds max_history_data limit
                                if len(self.DroneIDdataStructured[drone_index_old][12]) >= max_history_data:
                                    # print len(self.DroneIDdataStructured[drone_index_old][15])
                                    # print len(self.DroneIDdataStructured[drone_index_old][15])-max_history_data
                                    self.DroneIDdataStructured[drone_index_old][12] = self.DroneIDdataStructured[drone_index_old][12][len(self.DroneIDdataStructured[drone_index_old][12])-(max_history_data-1):len(self.DroneIDdataStructured[drone_index_old][12])]
                                    self.DroneIDdataStructured[drone_index_old][13] = self.DroneIDdataStructured[drone_index_old][14][len(self.DroneIDdataStructured[drone_index_old][13])-(max_history_data-1):len(self.DroneIDdataStructured[drone_index_old][13])]
                                    self.DroneIDdataStructured[drone_index_old][14] = self.DroneIDdataStructured[drone_index_old][15][len(self.DroneIDdataStructured[drone_index_old][14])-(max_history_data-1):len(self.DroneIDdataStructured[drone_index_old][14])]
                                    self.DroneIDdataStructured[drone_index_old][15] = self.DroneIDdataStructured[drone_index_old][15][len(self.DroneIDdataStructured[drone_index_old][15])-(max_history_data-1):len(self.DroneIDdataStructured[drone_index_old][15])]
                                # save old values: 'time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS'
                                self.DroneIDdataStructured[drone_index_old][12].append(self.DroneIDdataStructured[drone_index_old][1]) # 'time_since_epoch_oldS'
                                self.DroneIDdataStructured[drone_index_old][13].append(self.DroneIDdataStructured[drone_index_old][4]) # 'lat_oldS'
                                self.DroneIDdataStructured[drone_index_old][14].append(self.DroneIDdataStructured[drone_index_old][5]) # 'lng_oldS'
                                self.DroneIDdataStructured[drone_index_old][15].append(self.DroneIDdataStructured[drone_index_old][6]) # 'alt_oldS'
                                # Update values: time, epoch, lat, lng, alt, acc, fix, lnk, eng
                                self.DroneIDdataStructured[drone_index_old][0] = row[0] # time
                                self.DroneIDdataStructured[drone_index_old][1] = row[1] # epoch
                                self.DroneIDdataStructured[drone_index_old][4] = row[4] # lat
                                self.DroneIDdataStructured[drone_index_old][5] = row[5] # lng
                                self.DroneIDdataStructured[drone_index_old][6] = row[6] # alt
                                self.DroneIDdataStructured[drone_index_old][7] = row[7] # acc
                                self.DroneIDdataStructured[drone_index_old][8] = row[8] # fix
                                self.DroneIDdataStructured[drone_index_old][9] = row[9] # lnk
                                self.DroneIDdataStructured[drone_index_old][10] = row[10] # eng
                                # Update the raw entry
                                self.DroneIDdataRaw[drone_index_old] = line
                                self.drone_new_data_count = self.drone_new_data_count + 1
                        else:
                            if self.debug:
                                print "New", row[3], ", appending specific drone data"
                            if self.force_sim_to_real == True: row[11] = 0
                            else: row[11] = int(row[11]) # sim, not used for updating
                            # Make arrays for 'time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS'
                            row.append([])
                            row.append([])
                            row.append([])
                            row.append([])
                            # Add entry to structured
                            self.DroneIDdataRaw.append(line)
                            self.DroneIDdataStructured.append(row)
                            self.drone_count = self.drone_count + 1
            if self.debug:
                print 'Update done\n'
            return True

    def clear_history_data(self):
        """
        Clears the historical data
        Input: none
        Output: bool (True = deleted history, False = deletion failed due to no drones present in the system)
        """
        if self.drone_count > 0:
            for line in self.DroneIDdataStructured:
                line[12] = []
                line[13] = []
                line[14] = []
                line[15] = []

    def print_data(self):
        """
        Prints all the avaliable drone data (structured)
        Input: none
        Output: none but prints to the terminal
        """
        if self.drone_count > 0:
            print self.DroneIDdataStructured

    def print_raw(self):
        """
        Prints all the avaliable drone data (raw as directly from the download)
        Input: none
        Output: none but prints to the terminal
        """
        if self.drone_count > 0:
            print self.DroneIDdataRaw
    def print_CSV(self):
        """
        Prints all the avaliable drone data (CSV)
        Input: none
        Output: none but prints to the terminal
        """
        if self.drone_count > 0:
            print ",".join(self.DroneIDdataFields)
            for line in self.DroneIDdataRaw:
                print line

    def print_drone_pretty(self, drone_index):
        """
        Prints the data from a single drone in a easy redable format
        Input: index of a drone present in the data
        Output: none but prints to the terminal
        """
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
        """
        Prints the data format
        Input: none
        Output: none but prints to the terminal
        """
        print "time_stamp,time_since_epoch,id,name,lat,lon,alt,acc,fix,lnk,eng,sim"

    def print_description(self):
        """
        Prints a field description
        Input: none
        Output: none but prints to the terminal
        """
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
        """
        Returns the time stamp of the specified drone
        Input: index of a drone present in the data
        Output: drone time stamp
        """
        # GMT+1
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][0]
        else: return None

    def get_time_since_epoch(self, drone_index):
        """
        Returns the time stamp in EPOCH format of the specified drone
        Input: index of a drone present in the data
        Output: drone time stamp in EPOCH format
        """
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][1]
        else: return None

    def get_time_since_epoch_formatted_UTC(self, drone_index):
        """
        Returns the time stamp in UTC timezone of the specified drone
        Input: index of a drone present in the data
        Output: drone time stamp in UTC format
        """
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return datetime.datetime.fromtimestamp(self.DroneIDdataStructured[drone_index][1], pytz.UTC).strftime('%Y-%m-%d %H:%M:%S')
        else: return None

    def get_time_since_epoch_formatted_local(self, drone_index):
        """
        Returns the time stamp in local timezone of the specified drone
        Input: index of a drone present in the data
        Output: drone time stamp in local timezone
        """
        # https://www.epochconverter.com/
        if self.drone_count > drone_index and type(drone_index)==int:
            return datetime.datetime.fromtimestamp(self.DroneIDdataStructured[drone_index][1]).strftime('%Y-%m-%d %H:%M:%S')
        else: return None

    def get_id(self, drone_index):
        """
        Returns the id of the specified drone
        Input: index of a drone present in the data
        Output: drone id
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][2]
        else: return None

    def get_name(self, drone_index):
        """
        Returns the name of the specified drone
        Input: index of a drone present in the data
        Output: drone name
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][3]
        else: return None

    def get_lat(self, drone_index):
        """
        Returns the latitude of the specified drone
        Input: index of a drone present in the data
        Output: drone latitude
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][4]
        else: return None

    def get_lng(self, drone_index):
        """
        Returns the longitude of the specified drone
        Input: index of a drone present in the data
        Output: drone longitude
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][5]
        else: return None

    def get_alt_m(self, drone_index):
        """
        Returns the altitude of the specified drone
        Input: index of a drone present in the data
        Output: drone altitude
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][6]
        else: return None

    def get_alt_ft(self, drone_index):
        """
        Returns the altitude in feet of the specified drone
        Input: index of a drone present in the data
        Output: drone altitude in feet
        """
        # unit: feet
        return self.get_alt_m(drone_index)*3.28084

    def get_acc(self, drone_index):
        """
        Returns the accuracy of the specified drone
        Input: index of a drone present in the data
        Output: drone accuracy
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][7]
        else: return None

    def get_fix(self, drone_index):
        """
        Returns the fix type of the specified drone
        Input: index of a drone present in the data
        Output: drone fix type
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][8]
        else: return None

    def get_lnk(self, drone_index):
        """
        Returns the GSM link signal quality of the specified drone
        Input: index of a drone present in the data
        Output: drone GSM link signal quality (0-100%)
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][9]
        else: return None

    def get_eng(self, drone_index):
        """
        Returns the energy (battery) of the specified drone
        Input: index of a drone present in the data
        Output: drone energy (battery) (0-100%)
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][10]
        else: return None

    def get_sim(self, drone_index):
        """
        Returns the simulated field of the specified drone
        Input: index of a drone present in the data
        Output: drone simulated bool (True = simulated, False = real)
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index][11]
        else: return None

    def get_time_since_epoch_history(self, drone_index):
        """
        Returns the EPOCH time history inclusive the current last epoch time (not included in history array)
        Input: index of a drone present in the data
        Output: drone time stamp history in EPOCH format
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return_arr = self.DroneIDdataStructured[drone_index][12]
            return_arr.append(self.get_time_since_epoch(drone_index))
            return return_arr
        else: return None

    def get_lat_history(self, drone_index):
        """
        Returns the epoch time history inclusive the current last epoch time (not included in history array)
        Input: index of a drone present in the data
        Output: drone latitude history
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return_arr = self.DroneIDdataStructured[drone_index][13]
            return_arr.append(self.get_lat(drone_index))
            return return_arr
        else: return None
    def get_drones_all(self):
        """
        Returns all drones downloaded
        Input: none
        Output: all downloaded drones in structured format
        """
        if self.drone_count > 0:
            return self.DroneIDdataStructured
        return None

    def get_drones_real(self):
        """
        Returns the real drones downloaded
        Input: none
        Output: real downloaded drones in structured format
        """
        return_arr = []
        if self.drone_count > 0:
            for line in self.DroneIDdataStructured:
                if line[11] == 0: # sim = 0 = real drone
                    return_arr.append(line)
        return return_arr

    def get_drones_sim(self):
        """
        Returns the simulated drones downloaded
        Input: none
        Output: simulated downloaded drones in structured format
        """
        return_arr = []
        if self.drone_count > 0:
            for line in self.DroneIDdataStructured:
                if line[11] == 1: # sim = 1 = simulated drone
                    return_arr.append(line)
        return return_arr

    def get_drone_data(self, drone_index):
        """
        Returns data on a single drone
        Input: index of a drone present in the data
        Output: data on the specified drone
        """
        if self.drone_count > drone_index and type(drone_index)==int:
            return self.DroneIDdataStructured[drone_index]
        else: return None

    def get_drone_data_from_name(self, drone):
        """
        Returns data on a single drone from a name
        Input: name of a drone present in the data
        Output: data on the specified drone
        """
        if self.drone_count > 0:
            if drone != "" and type(drone)==str:
                for line in self.DroneIDdataStructured:
                    if line[3] == drone:
                        return line
                return []
            else: return None
        else: return None

    def get_drone_index_from_name(self, drone):
        """
        Returns the drone index from a name
        Input: name of a drone present in the data
        Output: drone index
        """
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
        """
        Returns the drone index from an ID
        Input: ID of a drone present in the data
        Output: drone index
        """
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

    if len(sys.argv) > 1:
        #print "Enough arguments provided"
        # Run self test
        # self test function not made yet

        # Define testdata
        LOG_TEST_DATA = [
            {
                'aid': 900,
                'lat': 55.399,
                'lng': 10.385,
                'alt': 50,
            },
            {
                'aid': 901,
                'lat': 55.399,
                'lng': 10.385,
                'alt': 100,
            },
            {
                'aid': 902,
                'lat': 55.399,
                'lng': 10.385,
                'alt': 50,
            },
            {
                'aid': 903,
                'lat': 55.399,
                'lng': 10.385,
                'alt': 100,
            }
        ]

        if sys.argv[1] == run_modes[0]:
            print 'Entering', run_modes[0],' mode'

            # Declare simulator module
            simulator_module = droneid_simulator(False)
            simulator_module.send_log_entry(LOG_TEST_DATA[2])

            # Declare DroneID module
            droneid_module = droneid_data()
            droneid_module.download_data()

            ctr = 0
            try:
                while True:
                    time.sleep(1)
                    ctr = ctr + 1
                    if ctr == 9:
                        LOG_TEST_DATA[2]['lat'] = LOG_TEST_DATA[2]['lat'] + 0.0001
                        LOG_TEST_DATA[2]['lng'] = LOG_TEST_DATA[2]['lng'] + 0.0001
                        LOG_TEST_DATA[2]['alt'] = LOG_TEST_DATA[2]['alt'] + 0.1
                        simulator_module.send_log_entry (LOG_TEST_DATA[2])

                        LOG_TEST_DATA[3]['lat'] = LOG_TEST_DATA[3]['lat'] - 0.0001
                        LOG_TEST_DATA[3]['lng'] = LOG_TEST_DATA[3]['lng'] - 0.0001
                        LOG_TEST_DATA[3]['alt'] = LOG_TEST_DATA[3]['alt'] + 0.1
                        simulator_module.send_log_entry (LOG_TEST_DATA[3])
                        ctr = 0
                    #droneid_module.update_data(3)
                    #droneid_module.print_raw()
                    #droneid_module.print_data()
                    print droneid_module.get_drones_sim()
            except KeyboardInterrupt:
                pass


    else:
        print "Proper use of script requires argument to set mode"
        print " Modes:", run_modes
        print " Example: python", sys.argv[0], run_modes[0]

# unused code - for fast copying
#droneid_module.print_data()
#droneid_module.print_drone_pretty(0)
# droneid_module.print_raw()
# droneid_module.print_data()

# time.sleep(2)
# simulator_module.send_log_entry (LOG_TEST_DATA[0])
# simulator_module.send_log_entry (LOG_TEST_DATA[1])
# droneid_module.update_data()
# droneid_module.print_raw()
# droneid_module.print_data()
#
# time.sleep(2)
# simulator_module.send_log_entry (LOG_TEST_DATA[0])
# simulator_module.send_log_entry (LOG_TEST_DATA[1])
# droneid_module.update_data()
# droneid_module.print_raw()
# droneid_module.print_data()

#print droneid_module.get_drone_data(0)
#print droneid_module.get_drone_data_from_name('000901')
#print droneid_module.get_drone_index_from_id(901)
#print droneid_module.get_drone_index_from_name('000901')
#print droneid_module.get_time_since_epoch_formatted_local(0)
#droneid_module.print_description()
#droneid_module.print_raw()
#droneid_module.print_CSV()
