#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-01-29 TL First version
2018-01-30 TL Refined version with safety measures for internet connection and empty response
---------- TL Expanded class
2018-03-13 TL Class almost done, missing update function but allows redownload but no remembering
2018-03-14 TL Made update function, but it is not tested because the receiver is down
"""

"""
Description:
This scripts downloads ADS-B data from a source provided by Kjeld Jensen (kjen@mmmi.sdu.dk) and parses it accordingly for easy use
The script support redownload using the 'download_data' method
License: BSD 3-Clause
Data source: https://droneid.dk/tobias/adsb.php
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
import pytz # timezones in the datetime format
#print 'Import done\n'

forever = 60*60*24*365*100  # 100 years excl. leap year
inf = 4294967295 # 32bit from 0
internet_connection_tries = 5
m_to_feet = 3.280839895
feet_to_m = 0.3048
deg_to_rad = 0.01745329252
rad_to_deg = 57.295779513

mps_to_kts = 1.9438444924574
kts_to_mps = 0.51444444444
mps_to_kph = 3.6
kph_to_mps = 0.27777777777778
mps_to_mph = 2.2369362920544
mph_to_mps = 0.44704
kph_to_mph = kph_to_mps*mps_to_mph
mph_to_kph = mph_to_mps*mps_to_kph

class adsb_data():
    def __init__(self, debug = False):
        self.internet_tester = internet_tools()
        self.url = 'https://droneid.dk/tobias/adsb.php'
        self.debug = debug
        self.aircraft_count = 0
        self.ADSBdataFieldsRaw = ['time_stamp','time_since_epoch','icao','flight','lat','lon','alt','track','speed']
        self.ADSBdataFields = ['time_stamp','time_since_epoch','icao','flight','lat','lon','alt','track','speed','time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS','track_oldS','speed_oldS']
        self.ADSBdataRaw = []
        self.ADSBdataStructured = []
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
        self.aircraft_count = 0
        self.ADSBdataRaw = []
        self.ADSBdataStructured = []
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
                self.ADSBdataRaw.append(line)

                csv_reader = csv.reader( [ line ] )
                for row in csv_reader:
                    row[1] = int(row[1]) # epoch time
                    row[4] = float(row[4]) # lat
                    row[5] = float(row[5]) # lng
                    row[6] = float(row[6]) # alt
                    row[7] = int(row[7]) # track
                    row[8] = int(row[8]) # speed
                    row.append([]) # time_since_epoch_oldS
                    row.append([]) # lat_oldS
                    row.append([]) # lng_oldS
                    row.append([]) # alt_oldS
                    row.append([]) # track_oldS
                    row.append([]) # speed_oldS
                    self.ADSBdataStructured.append(row)
        self.aircraft_count = itr
        if self.aircraft_count == 0:
            print "ADS-B receiver seems to be down; no data recieved"
        if self.debug:
            print "Entries:",self.aircraft_count
        response.close()
        if self.debug:
            print "Result read successfully"

        # Make JSON Data
        # for row in self.ADSBdataStructured:
        #     print row
    def update_data(self, max_history_data = inf):
        if self.debug:
            print '\nUpdate begun'
        self.aircraft_new_data_count = 0

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
                    row[4] = float(row[4]) # lat
                    row[5] = float(row[5]) # lng
                    row[6] = float(row[6]) # alt
                    row[7] = int(row[7]) # track
                    row[8] = int(row[8]) # speed

                    aircraft_index_old = self.get_aircraft_index(row[3]) # match aircraft name to see if it exists
                    if aircraft_index_old != None:
                        if self.debug:
                            print "Already seen", row[3], ", id:", aircraft_index_old
                        if self.ADSBdataStructured[1] < row[1]:
                            # New data received
                            # Remove data which exceeds max_history_data limit
                            if len(self.ADSBdataStructured[aircraft_index_old][9]) >= max_history_data:
                                # print len(self.ADSBdataStructured[aircraft_index_old][9])
                                # print len(self.ADSBdataStructured[aircraft_index_old][9])-max_history_data
                                self.ADSBdataStructured[aircraft_index_old][9] = self.ADSBdataStructured[aircraft_index_old][9][len(self.ADSBdataStructured[aircraft_index_old][9])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][9])]
                                self.ADSBdataStructured[aircraft_index_old][10] = self.ADSBdataStructured[aircraft_index_old][10][len(self.ADSBdataStructured[aircraft_index_old][10])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][10])]
                                self.ADSBdataStructured[aircraft_index_old][11] = self.ADSBdataStructured[aircraft_index_old][11][len(self.ADSBdataStructured[aircraft_index_old][11])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][11])]
                                self.ADSBdataStructured[aircraft_index_old][12] = self.ADSBdataStructured[aircraft_index_old][12][len(self.ADSBdataStructured[aircraft_index_old][12])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][12])]
                                self.ADSBdataStructured[aircraft_index_old][13] = self.ADSBdataStructured[aircraft_index_old][13][len(self.ADSBdataStructured[aircraft_index_old][13])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][13])]
                                self.ADSBdataStructured[aircraft_index_old][14] = self.ADSBdataStructured[aircraft_index_old][14][len(self.ADSBdataStructured[aircraft_index_old][14])-(max_history_data-1):len(self.ADSBdataStructured[aircraft_index_old][14])]
                            # save old values: 'time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS','track_oldS','speed_oldS'
                            self.ADSBdataStructured[aircraft_index_old][9].append(self.ADSBdataStructured[aircraft_index_old][1]) # time_since_epoch_oldS
                            self.ADSBdataStructured[aircraft_index_old][10].append(self.ADSBdataStructured[aircraft_index_old][4]) # lat_oldS
                            self.ADSBdataStructured[aircraft_index_old][11].append(self.ADSBdataStructured[aircraft_index_old][5]) # lng_oldS
                            self.ADSBdataStructured[aircraft_index_old][12].append(self.ADSBdataStructured[aircraft_index_old][6]) # alt_oldS
                            self.ADSBdataStructured[aircraft_index_old][13].append(self.ADSBdataStructured[aircraft_index_old][7]) # track_oldS
                            self.ADSBdataStructured[aircraft_index_old][14].append(self.ADSBdataStructured[aircraft_index_old][8]) # speed_oldS
                            # Update values: time, epoch, lat, lon, alt, track, speed
                            self.ADSBdataStructured[aircraft_index_old][0] = row[0] # time
                            self.ADSBdataStructured[aircraft_index_old][1] = row[1] # epoch
                            self.ADSBdataStructured[aircraft_index_old][4] = row[4] # lat
                            self.ADSBdataStructured[aircraft_index_old][5] = row[5] # lng
                            self.ADSBdataStructured[aircraft_index_old][6] = row[6] # alt
                            self.ADSBdataStructured[aircraft_index_old][7] = row[7] # track
                            self.ADSBdataStructured[aircraft_index_old][8] = row[8] # speed
                            # Update the raw entry
                            self.ADSBdataFieldsRaw[aircraft_index_old] = line
                            self.aircraft_new_data_count = self.aircraft_new_data_count + 1
                        else:
                            if self.debug:
                                print "New", row[3], ", appending specific aircraft data"
                            # Make arrays for 'time_since_epoch_oldS','lat_oldS','lng_oldS','alt_oldS'
                            row.append([]) # time_since_epoch_oldS
                            row.append([]) # lat_oldS
                            row.append([]) # lng_oldS
                            row.append([]) # alt_oldS
                            row.append([]) # track_oldS
                            row.append([]) # speed_oldS
                            # Add entry to structured
                            self.ADSBdataRaw.append(line)
                            self.ADSBdataStructured.append(row)
                            self.aircraft_count = self.aircraft_count + 1
        if self.debug:
            print 'Update done\n'
    def print_data(self):
        if self.aircraft_count > 0:
            print self.ADSBdataStructured
    def print_raw(self):
        if self.aircraft_count > 0:
            print self.ADSBdataRaw
    def print_CSV(self):
        if self.aircraft_count > 0:
            print ",".join(self.ADSBdataFields)
            for line in self.ADSBdataRaw:
                print line
    def print_aircraft_pretty(self, aircraft_index):
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            print "Name:       ", self.get_name(aircraft_index)
            print "ICAO:       ", self.get_icao_addr(aircraft_index)
            print "Timestamp:  ", self.get_time_stamp(aircraft_index)
            print "Epoch time:  ", self.get_time_since_epoch(drone_index),'- local:', datetime.datetime.fromtimestamp(self.get_time_since_epoch(aircraft_index)).strftime('%Y-%m-%d %H:%M:%S'),'- UTC:',datetime.datetime.fromtimestamp(self.get_time_since_epoch(aircraft_index), pytz.UTC).strftime('%Y-%m-%d %H:%M:%S')
            print "Lat:        ", self.get_lat(aircraft_index)
            print "Lng:        ", self.get_lng(aircraft_index)
            print "OSM link:   ", 'http://www.openstreetmap.org/?mlat=%s&mlon=%s&zoom=16' % (self.get_lat(aircraft_index),self.get_lng(aircraft_index))
            print "Google link:"  'http://maps.google.com/maps?q=%s+%s' % (self.get_lat(aircraft_index),self.get_lng(aircraft_index))
            print "Speed:      ", self.get_speed_kts(aircraft_index), "kts -", self.get_speed_mps(aircraft_index), "m/s -", self.get_speed_kph(aircraft_index), "k/h -", self.get_speed_mph(aircraft_index), "miles/h"
            print "Altitude:   ", self.get_alt_m(aircraft_index), "m -", self.get_alt_ft(aircraft_index), "ft"
            print "Track:      ", self.get_track_deg(aircraft_index), "° -", self.get_track_rad(aircraft_index), "rad\n"
    def print_format(self):
        #Format is $time_stamp,$time_since_epoch,$icao,$flight,$lat,$lon,$alt,$track,$speed\n
        print "time_stamp,time_since_epoch,icao,flight,lat,lon,alt,track,speed"
    def print_description(self):
        print """\nField description:
        time_stamp: Formatted time stamp
        time: seconds since Epoch
        icao: ICAO aircraft ID
        flight: Flight name (if available)
        lat: [Decimal degrees]
        lon: [Decimal degrees]
        alt: [m]
        trk: Track [degrees]
        hsp: Horizontal speed [m/s]"""
    def get_time_stamp(self, aircraft_index):
        # GMT+1
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][0]
        else: return None
    def get_time_since_epoch(self, aircraft_index):
        # https://www.epochconverter.com/
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][1]
        else: return None
    def get_time_since_epoch_formatted_UTC(self, aircraft_index):
        # https://www.epochconverter.com/
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return datetime.datetime.fromtimestamp(self.ADSBdataStructured[aircraft_index][1], pytz.UTC).strftime('%Y-%m-%d %H:%M:%S')
        else: return None
    def get_time_since_epoch_formatted_local(self, aircraft_index):
        # https://www.epochconverter.com/
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return datetime.datetime.fromtimestamp(self.ADSBdataStructured[aircraft_index][1]).strftime('%Y-%m-%d %H:%M:%S')
        else: return None
    def get_icao_addr(self, aircraft_index):
        # Use ex. World Aircraft Database to decode: https://junzisun.com/adb/
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][2]
        else: return None
    def get_name(self, aircraft_index):
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][3]
        else: return None
    def get_lat(self, aircraft_index):
        # unit: dot decimal
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][4]
        else: return None
    def get_lng(self, aircraft_index):
        # unit: dot decimal
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][5]
        else: return None
    def get_lon(self, aircraft_index): #same as get_lng
        # unit: dot decimal
        return self.get_lng(aircraft_index)
    def get_alt_m(self, aircraft_index):
        # unit: m
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][6]
        else: return None
    def get_alt_ft(self, aircraft_index):
        # unit: feet
        return self.get_alt_m(aircraft_index)*m_to_feet
    def get_track_deg(self, aircraft_index):
        # unit: degrees from north positive clockwise
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][7]
        else: return None
    def get_track_rad(self, aircraft_index):
        # unit: radians
        return self.get_track_deg(aircraft_index)*deg_to_rad
    def get_speed_mps(self, aircraft_index):
        # unit: m/s
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index][8]
        else: return None
    def get_speed_kts(self, aircraft_index):
        # unit: knots, kts
        return self.get_speed_mps(aircraft_index)*mps_to_kts
    def get_speed_kph(self, aircraft_index):
        # unit: m/s
        return self.get_speed_kts(aircraft_index)*mps_to_kph
    def get_speed_mph(self, aircraft_index):
        # unit: m/s
        return self.get_speed_mps(aircraft_index)*mps_to_mph
    def get_aircraft_all(self, aircraft_index):
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured
        return None
    def get_aircraft_data(self, aircraft_index):
        if self.aircraft_count > aircraft_index and type(aircraft_index)==int:
            return self.ADSBdataStructured[aircraft_index]
        else: return None
    def get_aircraft_data_from_name(self, aircraft):
        if self.aircraft_count > 0:
            if aircraft != "" and type(aircraft)==str:
                for line in self.ADSBdataStructured:
                    if line[3] == aircraft:
                        return line
                return []
            else: return None
        else: return None
    def get_aircraft_index(self, aircraft):
        if self.aircraft_count != "" and type(aircraft)==str:
            itr = 0
            for line in self.ADSBdataStructured:
                if line[3] == aircraft:
                    return itr
                itr = itr+1
            return None
        else: return None
    def check_input_aircraft_index(self, aircraft_index):
        # Checks if the aircraft_index has been defined
        # Currently not used.
        try:
            aircraft_index
        except NameError:
            print "Input not defined, terminating"
            sys.exit()
        else:
            return
    def save_CSV_file(self, include_history = False):
        now = datetime.datetime.now()
        file_name = 'ADS-B_%d-%02d-%02d-%02d-%02d.csv' % (now.year, now.month, now.day, now.hour, now.minute)
        output_file_CSV = open(file_name, 'w')
        output_writer_CSV = csv.writer(output_file_CSV)
        output_writer_CSV.writerow(self.ADSBdataFields)
        for line in self.ADSBdataStructured:
            if include_history == True:
                output_writer_CSV.writerow(line)
            else:
                output_writer_CSV.writerow(line[0:9]) # the rest of the fields only include history data
        output_file_CSV.close()

def self_test():
    adsb_module_test = adsb_data(False)
    adsb_module_test.download_data()
    name = adsb_module_test.get_name(0)
    if adsb_module_test.get_aircraft_data_from_name(name) == adsb_module_test.get_aircraft_data(adsb_module_test.get_aircraft_index(name)):
        if adsb_module_test.get_time_stamp(forever) == None:
            print "Self test PASSED\n"
            return
    print "Self test not PASSED, terminating\n"
    sys.exit()

if __name__ == '__main__':
    # Run self test
    self_test()

    adsb_module = adsb_data(False)

    adsb_module.download_data()

    #adsb_module.print_raw()
    #print "\n\n"
    #adsb_module.print_CSV()
    #adsb_module.print_data()
    adsb_module.save_CSV_file()

    # adsb_module.update_data()
    #adsb_module.print_aircraft_pretty(0)
    # adsb_module.print_aircraft_pretty(1)
    # adsb_module.print_aircraft_pretty(2)
    # adsb_module.print_aircraft_pretty(3)
    # adsb_module.print_aircraft_pretty(4)
    #
    # adsb_module.print_aircraft_pretty(adsb_module.get_aircraft_index("VKG1305"))
    #
    # adsb_module.check_input_aircraft_index(1)

    #print adsb_module.get_aircraft_data_from_name("VKG1305")
    #print adsb_module.get_aircraft_data(adsb_module.get_aircraft_index("BAW798H"))
