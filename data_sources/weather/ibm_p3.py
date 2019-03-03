#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
2018-01-05 TL First version
"""

"""
Description:
None
see: http://2017.compciv.org/guide/topics/python-standard-library/csv.html
see: https://docs.scipy.org/doc/numpy-1.13.0/user/basics.creation.html
For treating empy cells (usable for SDU data): https://www.youtube.com/watch?v=yQsOFWqpjkE
License: BSD 3-Clause
"""

### Import start
from math import floor, ceil
import csv
import numpy as np
from bokeh.io import output_file, show, export_svgs, export_png
from bokeh.layouts import gridplot, column, widgetbox, layout
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, Legend, LegendItem, Label, LabelSet, Arrow, NormalHead, OpenHead
from bokeh.models.widgets import Div
#from bokeh.models import axes
from bokeh.colors import RGB
from datetime import datetime
import pandas as pd

import sys

from svglib.svglib import svg2rlg
from reportlab.graphics import renderPDF
### Import end

### Define start
ICING_TEMP_DIFF = 2.7 # the allowed difference between the surface temperature and dewpoint temperature
ICING_MAX_TEMP  = 0 # just a bit over 0 degrees Celcius # old value from M.Sc 0 deg C
ICING_MIN_TEMP  = -20 # unit: degrees Celcius # old value from M.Sc -15 deg C # new value includes clear, rime, and mixed

CONV_10kts2mps = 5.14 # 10kts more than wind speed based on the tower at HCA Airport
CONV_kph2mps = 1/3.6

USE_TEMP_APPARENT               = False
USE_ICING_CONDITION_CONDITION   = True
USE_ICING_CONDITION             = True
USE_PRECIPITATION_CONDITION     = True
USE_SNOWFALL_CONDITION          = True

SHOW_WEBPAGE = True
GENERATE_SVG_et_PDF = True
### Define end

### Class start
class ibm_weather_csv():
    def __init__(self, inFileName, inCity, inYear, inWindSpeedOperationMax, inWindGustOperationMax, inTemperatureOperationMin, inTemperatureOperationMax, inPrecipitationMax, inSnowfallMax, debug = False):
        self.fileName = inFileName;
        self.city = inCity
        self.year = inYear

        # Save UAV limits as class variables
        self.WindSpeedOperationMax = inWindSpeedOperationMax
        self.WindGustOperationMax = inWindGustOperationMax
        self.TempOperationMin = inTemperatureOperationMin
        self.TempOperationMax = inTemperatureOperationMax
        self.PrecipitationMax = inPrecipitationMax
        self.SnowfallMax = inSnowfallMax

        # Prepare containers for the data
        self.DateSGMT = []

        self.WindSpeedKphS = []
        self.WindSpeedMpsS = []
        self.WindGustSurfaceKphS = []
        self.WindGustSurfaceMpsS = []
        self.WindDirectionDegreesS = []

        self.TempSurfaceCS = []
        self.TempApparentCS = []
        self.TempWindChillCS = []
        self.TempSurfaceDewpointCS = []

        self.PrecipitationPreviousHourCmS = []
        self.SnowfallCmS = []

        # Bookkeeping values
        self.samples = 0
        self.days = 0

        # Bokeh variables
        self.plotWidth = 800
        self.plotHeight = 400

        # Print debug value
        self.debugText = debug

    def reset(self):
        self.fileName = '';
        self.city = ''
        self.year = np.nan

        self.WindSpeedOperationMax = np.nan
        self.WindGustOperationMax = np.nan
        self.TempOperationMin = np.nan
        self.TempOperationMax = np.nan

        self.DateSGMT = []

        self.WindSpeedKphS = []
        self.WindSpeedMpsS = []
        self.WindGustSurfaceKphS = []
        self.WindGustSurfaceMpsS = []
        self.WindDirectionDegreesS = []

        self.TempSurfaceCS = []
        self.TempApparentCS = []
        self.TempWindChillCS = []
        self.TempSurfaceDewpointCS = []

        self.PrecipitationPreviousHourCmS = []
        self.SnowfallCmS = []

        self.samples = 0
        self.days = 0

    def loadCSV(self):
        if self.debugText:
            print("\nLocation:", self.city)
            print('[%s] Attempting to open data file:' % self.city, self.fileName)
        with open(self.fileName) as csvfile:
        #with open('DronePlanning/sec.csv') as csvfile:
            if self.debugText:
                print('[%s] Data file opened, attempting data load' % self.city)
            readCSV = csv.DictReader(csvfile, delimiter=',')
            for row in readCSV:
                # Date load - format 01/01/2016 00:00:00
                DateGMT = datetime.strptime(row['DateHrGmt'], '%m/%d/%Y %H:%M:%S')
                self.DateSGMT.append(DateGMT)

                # Wind speed load
                WindSpeedKph = float(row['WindSpeedKph'])
                self.WindSpeedKphS.append(WindSpeedKph)

                SurfaceWindGustsKph = float(row['SurfaceWindGustsKph'])
                self.WindGustSurfaceKphS.append(SurfaceWindGustsKph)

                WindDirectionDegrees = float(row['WindDirectionDegrees'])
                self.WindDirectionDegreesS.append(WindDirectionDegrees)

                # Temperature load
                SurfaceTempC = float(row['SurfaceTemperatureCelsius'])
                self.TempSurfaceCS.append(SurfaceTempC)

                ApparentTemperatureC = float(row['ApparentTemperatureCelsius'])
                self.TempApparentCS.append(ApparentTemperatureC)

                WindChillTemperatureC = float(row['WindChillTemperatureCelsius'])
                self.TempWindChillCS.append(WindChillTemperatureC)

                #SurfaceDewpointTemperatureCelsius
                SurfaceDewpointTemperatureC = float(row['SurfaceDewpointTemperatureCelsius'])
                self.TempSurfaceDewpointCS.append(SurfaceDewpointTemperatureC)

                PrecipitationPreviousHourCm = float(row['PrecipitationPreviousHourCentimeters'])
                self.PrecipitationPreviousHourCmS.append(PrecipitationPreviousHourCm)

                SnowfallCm = float(row['SnowfallCentimeters'])
                self.SnowfallCmS.append(SnowfallCm)
            if self.debugText:
                print('[%s] Data loaded' % self.city)

        #x = np.arange(len(SurfaceTempS))

        self.samples = len(self.DateSGMT)
        self.days = int(round(len(self.DateSGMT)/24))
        if self.debugText:
            print('[%s] Samples:' % self.city, str(self.samples))
            print('[%s] Days:' % self.city, str(self.days))

        # convert to the more used m/s
        self.WindSpeedMpsS = np.multiply(self.WindSpeedKphS, CONV_kph2mps)
        self.WindGustSurfaceMpsS = np.multiply(self.WindGustSurfaceKphS, CONV_kph2mps)
        #WindSpeedMpsS = [x / 3.6 for x in WindSpeedKphS] #note that the list contains floats, use calc above
        #print(WindSpeedMpsS)

    def convert_time_to_index(self, year, month, date, hour):
        # Since I only have data from 2016 the year is just ignored
        date_obj = datetime.strptime('%02d/%02d/%04d %02d' % (date, month, year, hour), '%m/%d/%Y %H')
        for i in range(len(self.DateSGMT)):
            if self.DateSGMT[i] == date_obj:
                return i
        return None

    def get_wind_speed(self, index):
        return self.WindSpeedMpsS[index]
    def get_wind_gust(self, index):
        return self.WindGustSurfaceMpsS[index]
    def get_wind_direction(self, index):
        return self.WindDirectionDegreesS[index]
    def get_temp(self, index):
        return self.TempSurfaceCS[index]
    def get_temp_dewpoint(self, index):
        return self.TempSurfaceDewpointCS[index]
    def get_precipitation(self, index):
        return self.PrecipitationPreviousHourCmS[index]
    def get_snowfall(self, index):
        return self.SnowfallCmS[index]



    ## Check condition methods
    def check_condition_windspeed(self, sample_nr):
        # Info: Checks if the wind speed is exceeding the limit for the provided sample number
        # Input: sample number
        # Output: true = satisfies wind speed limit, false = does not satisfy wind speed limit

        # self.WindSpeedMpsS[i] > self.WindSpeedOperationMax or self.WindGustSurfaceMpsS[i] > self.WindSpeedOperationMax
        if self.WindSpeedMpsS[sample_nr] > self.WindSpeedOperationMax:
            #print("WIND exceed")
            return False
        if USE_ICING_CONDITION_CONDITION:
            if self.WindGustSurfaceMpsS[sample_nr] > self.WindGustOperationMax:
                #print("GUST exceed")
                return False
        return True

    def check_condition_temp(self, sample_nr):
        # Info: Checks if the temperature is exceeding the limit for the provided sample number
        # Input: sample number
        # Output: true = satisfies temperature limits, false = does not satisfy temperature limits
        if USE_TEMP_APPARENT:
            if self.TempApparentCS[sample_nr] < self.TempOperationMin:
                return False
            if self.TempApparentCS[sample_nr] > self.TempOperationMax:
                return False
            return True
        else:
            if self.TempSurfaceCS[sample_nr] < self.TempOperationMin:
                return False
            if self.TempSurfaceCS[sample_nr] > self.TempOperationMax:
                return False
            return True
    def check_condition_icing(self, sample_nr):
        # Info: Checks if there is chance of icing is exceeding the limit for the provided sample number
        # Input: sample number
        # Output: true = no icing risk, false = icing possiblity
        # true = no icing, false = icing possiblity
        if USE_ICING_CONDITION:
            diff_SurfaceTem_SurfaceDewpointTemp = abs(self.TempSurfaceCS[sample_nr] - self.TempSurfaceDewpointCS[sample_nr])
            # if self.TempSurfaceCS[sample_nr] < 0:
            #     print "Icing: date ", self.DateSGMT[sample_nr], "diff:", diff_SurfaceTem_SurfaceDewpointTemp, "temp:", self.TempSurfaceCS[sample_nr]
            if diff_SurfaceTem_SurfaceDewpointTemp < ICING_TEMP_DIFF and (self.TempSurfaceCS[sample_nr] < ICING_MAX_TEMP and self.TempSurfaceCS[sample_nr] > ICING_MIN_TEMP):
                return False
            else:
                return True
        else:
            return True
    def check_condition_precipitation(self, sample_nr):
        # true = can fly, false = cannot fly due to rain
        if USE_PRECIPITATION_CONDITION:
            if self.PrecipitationPreviousHourCmS[sample_nr] > self.PrecipitationMax:
                return False
            else:
                return True
        else:
            return True
    def check_condition_snowfall(self, sample_nr):
        # true = can fly, false = cannot fly due to rain
        if USE_SNOWFALL_CONDITION:
            if self.SnowfallCmS[sample_nr] > self.SnowfallMax:
                return False
            else:
                return True
        else:
            return True
    def check_condition_all(self, sample_nr):
        if self.check_condition_windspeed(sample_nr) and self.check_condition_temp(sample_nr) and self.check_condition_icing(sample_nr) and self.check_condition_precipitation(sample_nr) and self.check_condition_snowfall(sample_nr):
            return True
        else:
            return False
    def check_condition_all_with_type(self, sample_nr):
        # no_fly = [ [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ], [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ] ]

        res_arr = []
        res_arr.append(self.check_condition_windspeed(sample_nr))
        res_arr.append(self.check_condition_icing(sample_nr))
        res_arr.append(self.check_condition_precipitation(sample_nr))
        res_arr.append(self.check_condition_snowfall(sample_nr))
        res_arr.append(self.check_condition_temp(sample_nr))
        #print(res_arr)

        res_val = 0
        for value in res_arr:
            res_val += not value
        #print(res_val)

        condition_type = 0
        if res_val == 0:
            condition_type = 0 # within conditions
        elif res_val == 5:
            condition_type = 1 # all exceeding
        elif res_val >= 2:
            condition_type = 2 # multiple problems
        elif res_arr[0] == False:
            condition_type = 3 # wind problem (exceeding)
        elif res_arr[1] == False:
            condition_type = 4 # icing risk
        elif res_arr[2] == False:
            condition_type = 5 # rain problem (exceeding)
        elif res_arr[3] == False:
            condition_type = 6 # snowfall problem (exceeding)
        elif res_arr[4] == False:
            condition_type = 7 # temp problem (exceeding)
        #print condition_type
        return condition_type

    ## Plot methods
    def createTimePlot(self, inTitle, inXAxis, inYAxis):
        return figure(
            tools="pan,box_zoom,reset,save,hover", #hover
            title="%s" % inTitle,
            x_axis_type='datetime',
            x_axis_label='%s' % inXAxis, y_axis_label='%s' % inYAxis,
            plot_height = self.plotHeight, plot_width = self.plotWidth,
        )
    ## Analyzer methods
    def analyseWind(self):
        # Calculate percentage statistics - simple how many samples are over the limit
        print("\n[%s] Wind analysis" % self.city)
        aboveThreshold = 0
        for i in range(self.samples):
        	if self.check_condition_windspeed(i) == False:
        		aboveThreshold += 1
                #print self.DateSGMT[i], self.WindSpeedMpsS[i]
        if self.debugText:
            print('[%s] Number of samples above %d m/s = ' % (self.city, self.WindSpeedOperationMax), aboveThreshold)
            print('[%s] Percentage of samples above %d m/s = ' % (self.city, self.WindSpeedOperationMax), aboveThreshold/(self.samples * 1.0)*100.0, '%')

        # Calculate consecutive periods with max conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0
        periodsAbove = []

        periods = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_condition_windspeed(i) == False:
                in_period_count += 1
            else:
                if in_period_count > 0:
                    hours = in_period_count
                    periods.append (hours)

                    if hours <= 1.0:
                        per_1h_count += 1
                    elif hours <= 2.0:
                        per_2h_count += 1
                    elif hours <= 3.0:
                        per_3h_count += 1
                    elif hours <= 4.0:
                        per_4h_count += 1
                    else:
                        periodsAbove.append (hours)

                in_period_count = 0
        if self.debugText:
            print('[%s] Number of periods with reports above %d m/s = ' % (self.city, self.WindSpeedOperationMax), len(periods))

            print('0-1 hour : ', per_1h_count)
            print('1-2 hours: ', per_2h_count)
            print('2-3 hours: ', per_3h_count)
            print('3-4 hours: ', per_4h_count)
            print("> 4 hours: ",len(periodsAbove))

        #

        noWindDays = 0
        # init array
        hoursOfWind = []
        for i in range(25): # 25 instead of 24 since 'nothing' should also be included
            hoursOfWind.append(0)
        # loop through the data
        for day in range(self.days): # itr days
            extreme_hour_count = 0
            for sample in range(24): # itr samples
                if self.check_condition_windspeed(day*24+sample) == False:
                    extreme_hour_count += 1
            if extreme_hour_count >= 2:
                hoursOfWind[extreme_hour_count] +=1
            elif extreme_hour_count == 0:
                noWindDays += 1
        #print hoursOfWind

        return [periods, hoursOfWind]

    def analyseTemperature(self):
        # Calculate percentage statistics - simple how many samples are over the limit
        print("\n[%s] Temperature analysis" % self.city)
        belowThreshold = 0
        for i in range(self.samples):
        	if self.check_condition_temp(i) == False:
        		belowThreshold += 1
                #print self.DateSGMT[i], self.TempApparentCS[i]
        if self.debugText:
            print('[%s] Number of samples below %d °C = ' % (self.city, self.TempOperationMin), belowThreshold)
            print('[%s] Percentage of samples below %d °C = ' % (self.city, self.TempOperationMin), belowThreshold/(self.samples * 1.0)*100.0, '%')

        # Calculate consecutive periods with min conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0
        periodsAbove = []

        periods = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_condition_temp(i) == False:
                in_period_count += 1
            else:
                if in_period_count > 0:
                    hours = in_period_count
                    periods.append (hours)

                    if hours <= 1.0:
                        per_1h_count += 1
                    elif hours <= 2.0:
                        per_2h_count += 1
                    elif hours <= 3.0:
                        per_3h_count += 1
                    elif hours <= 4.0:
                        per_4h_count += 1
                    else:
                        periodsAbove.append (hours)

                in_period_count = 0
        if self.debugText:
            print('[%s] Number of periods with reports below %d °C = ' % (self.city, self.TempOperationMin), len(periods))

            print('0-1 hour : ', per_1h_count)
            print('1-2 hours: ', per_2h_count)
            print('2-3 hours: ', per_3h_count)
            print('3-4 hours: ', per_4h_count)
            print("> 4 hours: ",len(periodsAbove))

        return periods
    def analyseCombined(self):
        # Calculate percentage statistics - simple how many samples are over the limit
        print("\n[%s] Combined weather analysis - single city" % self.city)
        excedingConditions = 0
        for i in range(self.samples):
        	if self.check_condition_all(i) == False:
        		excedingConditions += 1
                #print(self.DateSGMT[i], self.WindSpeedMpsS[i])
        if self.debugText:
            print('[%s] Number of samples EXCEEDING conditions = ' % self.city, excedingConditions)
            print('[%s] Percentage of samples EXCEEDING conditions = ' % self.city, excedingConditions/(self.samples * 1.0)*100.0, '%')
            print('[%s] Number of samples WITHIN conditions = ' % self.city, self.samples-excedingConditions)
            print('[%s] Percentage of samples WITHIN conditions = ' % self.city, (self.samples-excedingConditions)/(self.samples * 1.0)*100.0, '%')

        # Calculate consecutive periods with max conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0
        periodsAbove = []

        periods = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_condition_all(i) == False:
                in_period_count += 1
                #print self.DateSGMT[i]
            else:
                if in_period_count > 0:
                    #print in_period_count
                    hours = in_period_count
                    periods.append (hours)

                    if hours <= 1.0:
                        per_1h_count += 1
                    elif hours <= 2.0:
                        per_2h_count += 1
                    elif hours <= 3.0:
                        per_3h_count += 1
                    elif hours <= 4.0:
                        per_4h_count += 1
                    else:
                        periodsAbove.append (hours)

                in_period_count = 0
        if self.debugText:
            print('[%s] Number of periods with reports exceeding conditions = ' % self.city, len(periods))

            print('0-1 hour : ', per_1h_count)
            print('1-2 hours: ', per_2h_count)
            print('2-3 hours: ', per_3h_count)
            print('3-4 hours: ', per_4h_count)
            print("> 4 hours: ",len(periodsAbove))

        #

        withinDays = 0
        # init array
        hoursExcedingConditions = []
        for i in range(25): # 25 instead of 24 since 'nothing' should also be included
            hoursExcedingConditions.append(0)
        # loop through the data
        for day in range(self.days): # itr days
            extreme_hour_count = 0
            for sample in range(24): # itr samples
                if self.check_condition_all(day*24+sample) == False:
                    extreme_hour_count += 1
            if extreme_hour_count >= 2:
                hoursExcedingConditions[extreme_hour_count] +=1
                # if extreme_hour_count == 24:
                #     print "day: ", day
            elif extreme_hour_count == 0:
                withinDays += 1
        #print hoursExcedingConditions


        #
        # no_fly = [ [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ], [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ] ]
        combind_results = []
        day_result = []
        cur_start_hour = 0
        cur_hour = 0
        last_result = 0
        cur_result = 0
        #for day in range(self.days): # itr days
        for day in range(self.days): # itr days
            cur_start_hour = 0
            last_result = self.check_condition_all_with_type(day*24)
            day_result = []
            for sample in range(1, 24): # itr samples
                cur_hour = sample
                cur_result = self.check_condition_all_with_type(day*24+cur_hour)
                if cur_result != last_result:
                    day_result.append([cur_start_hour, cur_hour, last_result])
                    last_result = cur_result
                    cur_start_hour = cur_hour
                #print self.DateSGMT[sample]
                if cur_hour == 23:
                    day_result.append([cur_start_hour, cur_hour+1, last_result])
            combind_results.append(day_result)
        #print combind_results

        return [periods, hoursExcedingConditions, combind_results]

### Class end - Functions start

def analyseCombined_2cities(obj_c1, obj_c2):
    # NOTE that it is assumed that both objetcs have the same number of samples

    # Calculate percentage statistics - simple how many samples are over the limit
    print("\n[BOTH] Combined weather analysis - %s and %s " % (obj_c1.city, obj_c2.city))
    excedingConditions = 0
    for i in range(obj_c1.samples):
        if obj_c1.check_condition_all(i) == False or obj_c2.check_condition_all(i) == False:
            excedingConditions += 1
            #print(self.DateSGMT[i], self.WindSpeedMpsS[i])
    if obj_c1.debugText:
        print('[BOTH] Number of samples EXCEEDING conditions = ', excedingConditions)
        print('[BOTH] Percentage of samples EXCEEDING conditions = ', excedingConditions/(obj_c1.samples * 1.0)*100.0, '%')
        print('[BOTH] Number of samples WITHIN conditions = ', obj_c1.samples-excedingConditions)
        print('[BOTH] Percentage of samples WITHIN conditions = ', (obj_c1.samples-excedingConditions)/(obj_c1.samples * 1.0)*100.0, '%')

    # Calculate consecutive periods with max conditions
    per_1h_count = 0
    per_2h_count = 0
    per_3h_count = 0
    per_4h_count = 0
    periodsAbove = []

    periods = []
    in_period_count = 0

    for i in range(obj_c1.samples):
        if obj_c1.check_condition_all(i) == False or obj_c2.check_condition_all(i) == False:
            in_period_count += 1
            #print self.DateSGMT[i]
        else:
            if in_period_count > 0:
                #print in_period_count
                hours = in_period_count
                periods.append (hours)

                if hours <= 1.0:
                    per_1h_count += 1
                elif hours <= 2.0:
                    per_2h_count += 1
                elif hours <= 3.0:
                    per_3h_count += 1
                elif hours <= 4.0:
                    per_4h_count += 1
                else:
                    periodsAbove.append (hours)

            in_period_count = 0
    if obj_c1.debugText:
        print('[BOTH] Number of periods with reports exceeding conditions = ', len(periods))

        print('0-1 hour : ', per_1h_count)
        print('1-2 hours: ', per_2h_count)
        print('2-3 hours: ', per_3h_count)
        print('3-4 hours: ', per_4h_count)
        print("> 4 hours: ",len(periodsAbove))

    #

    withinDays = 0
    # init array
    hoursExcedingConditions = []
    for i in range(25): # 25 instead of 24 since 'nothing' should also be included
        hoursExcedingConditions.append(0)
    # loop through the data
    for day in range(obj_c1.days): # itr days
        extreme_hour_count = 0
        for sample in range(24): # itr samples
            if obj_c1.check_condition_all(day*24+sample) == False or obj_c2.check_condition_all(day*24+sample) == False:
                extreme_hour_count += 1
        if extreme_hour_count >= 2:
            hoursExcedingConditions[extreme_hour_count] +=1
            # if extreme_hour_count == 24:
            #     print "day: ", day
        elif extreme_hour_count == 0:
            withinDays += 1
    #print hoursExcedingConditions

    #
    # no_fly = [ [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ], [ [0,2,0], [2,14,1], [14,24,2] ],[ [0,13,0], [13,15,2], [15,24,1] ], [ [0,5,0], [5,7,1], [7,24,2] ], [ [0,13,0], [13,17,2], [17,24,1] ] ]
    combind_results = []
    day_result = []
    cur_start_hour = 0
    cur_hour = 0
    last_result = 0
    cur_result = 0
    #for day in range(self.days): # itr days
    for day in range(obj_c1.days): # itr days
        cur_start_hour = 0

        first_result_c1c2 = [obj_c1.check_condition_all_with_type(day*24), obj_c2.check_condition_all_with_type(day*24)]
        # Logic to handle that value 0 = within conditions etc.
        if first_result_c1c2[0] == 0 and first_result_c1c2[1] == 0: # all within
            last_result = first_result_c1c2[0]
        elif first_result_c1c2[0] == 0:
            last_result = first_result_c1c2[1]
        elif first_result_c1c2[1] == 0:
            last_result = first_result_c1c2[0]
        elif first_result_c1c2[0] == first_result_c1c2[1]:
            last_result = first_result_c1c2[0]
        else:
            last_result = 1 # multiple exceeding

        day_result = []
        for sample in range(1,24): # itr samples
            cur_hour = sample

            cur_result_c1c2 = [obj_c1.check_condition_all_with_type(day*24+cur_hour), obj_c2.check_condition_all_with_type(day*24+cur_hour)]
            # Logic to handle that value 0 = within conditions etc.
            if cur_result_c1c2[0] == 0 and cur_result_c1c2[1] == 0: # all within
                cur_result = cur_result_c1c2[0]
            elif cur_result_c1c2[0] == 0:
                cur_result = cur_result_c1c2[1]
            elif cur_result_c1c2[1] == 0:
                cur_result = cur_result_c1c2[0]
            elif cur_result_c1c2[0] == cur_result_c1c2[1]:
                cur_result = cur_result_c1c2[0]
            else:
                cur_result = 1 # multiple exceeding

            if cur_result != last_result:
                #day_result.append([cur_start_hour, cur_hour, last_result])
                day_result.append([cur_start_hour, cur_hour, last_result])
                last_result = cur_result
                cur_start_hour = cur_hour
            #print obj_c1.DateSGMT[sample]
            if cur_hour == 23:
                day_result.append([cur_start_hour, cur_hour+1, last_result])
        combind_results.append(day_result)
    #print combind_results

    return [periods, hoursExcedingConditions, combind_results]


def export_plot__png_eps_pdf(plot_to_export, plot_filename):
    plot_filename_png = plot_filename + '.png'
    plot_filename_svg = plot_filename + '.svg'
    plot_filename_pdf = plot_filename + '.pdf'
    export_png(plot_to_export, filename=plot_filename_png)
    plot_to_export.output_backend = "svg"
    export_svgs(plot_to_export, filename=plot_filename_svg)
    svg_plot = svg2rlg(plot_filename_svg)
    renderPDF.drawToFile(svg_plot, plot_filename_pdf)

def latex_plot_font(plot):
    # Set LaTex font ('cmr' but system only has 'cmr10')
    plot.xaxis.axis_label_text_font = "cmr10"
    plot.xaxis.major_label_text_font = "cmr10"
    plot.yaxis.axis_label_text_font = "cmr10"
    plot.yaxis.major_label_text_font = "cmr10"
    plot.legend.label_text_font = 'cmr10'

def change_plot_axis_names(plot, xaxis_text, yaxis_text):
    # Change the name of the x- and y-axis
    plot.xaxis.axis_label = xaxis_text
    plot.yaxis.axis_label = yaxis_text

def custom_legend(plot):
    plot.legend.orientation = "horizontal"
    plot.legend.label_text_font_size = "10px"
    new_legend = plot.legend[0]
    plot.legend[0].plot = None
    plot.add_layout(new_legend, 'above')
    plot.legend.glyph_width = 15
    plot.legend.glyph_height = 15
    plot.legend.padding = 3
    plot.legend.margin = 5

def draw_quads(plot, results_combined_arr, interval):
    for date_itr in range(len(results_combined_arr)):
          for internal_itr in range(len(results_combined_arr[date_itr])):
              #if results_combined_arr[date_itr][internal_itr][2] == 0: # 0 = palegreen color = within conditions
                   #plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], color=RGB(93,255,156)) # palegreen RGB(93,255,156)
               if results_combined_arr[date_itr][internal_itr][2] == 1: # 1 = red color = all exceeding
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="All", color=RGB(215,48,39)) # red, old: red RGB(255,0,0)
               if results_combined_arr[date_itr][internal_itr][2] == 2: # 1 = red color = all exceeding
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Multiple", color=RGB(244,109,67)) # orange, old: red RGB(255,0,0)
               if results_combined_arr[date_itr][internal_itr][2] == 3: # 2 = orange color = wind exceeding
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Wind or gust", color=RGB(253,174,97)) # light orange, old: orange (255,168,0)
               if results_combined_arr[date_itr][internal_itr][2] == 4: # 3 = yellow color = icing risk
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Icing", color=RGB(254,224,144)) # yellow, old: brown or saddlebrown RGB(146,66,22)
               if results_combined_arr[date_itr][internal_itr][2] == 5: # 3 = light blue color = rain risk
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Percipitation", color=RGB(171,217,233)) # light blue, old: hotpink RGB(255,0,175)
               if results_combined_arr[date_itr][internal_itr][2] == 6: # 5 = blue color = snowfall risk
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Snowfall", color=RGB(116,173,209)) # blue, old: magenta RGB(255,0,255)
               if results_combined_arr[date_itr][internal_itr][2] == 7: # 6 = dark blue = temp exceeding
                   plot.quad(left=interval['start'][date_itr],right=interval['end'][date_itr],bottom=results_combined_arr[date_itr][internal_itr][0], top=results_combined_arr[date_itr][internal_itr][1], legend="Temperature", color=RGB(69,117,180)) # dark blue, old: royalblue RGB(91,13,223)

### Functions end - Main start

if __name__ == '__main__':

    UAV = {
        'cumulus': {
            'WindSpeedOperationMax': 12, # unit: m/s
            'WindGustOperationMax': 12+CONV_10kts2mps, # unit: m/s
            'TempOperationMin': -20, # unit: degrees Celcius
            'TempOperationMax': 45, # unit: degrees Celcius
            'PrecipitationMax': 0.76, # unit: cm # intensity: moderate
            'SnowfallMax': 0.5 # unit: cm # intensity: light
        },
        'wingcopter': {
            'WindSpeedOperationMax': 15, # unit: m/s
            'WindGustOperationMax': 20, # unit: m/s
            'TempOperationMin': -20, # unit: degrees Celcius
            'TempOperationMax': 45, # unit: degrees Celcius
            'PrecipitationMax': 0.76, # unit: cm # intensity: moderate
            'SnowfallMax': 4 # unit: cm # intensity: light
        },
        'volocopter': {
            'WindSpeedOperationMax': 8.6, # unit: m/s
            'WindGustOperationMax': 8.6+CONV_10kts2mps, # unit: m/s
            'TempOperationMin': -20, # unit: degrees Celcius
            'TempOperationMax': 45, # unit: degrees Celcius
            'PrecipitationMax': 0.76, # unit: cm # intensity: moderate
            'SnowfallMax': 4 # unit: cm # intensity: light
        },
        'DJIP4': {
            'WindSpeedOperationMax': 10, # unit: m/s
            'WindGustOperationMax': 10+CONV_10kts2mps, # unit: m/s
            'TempOperationMin': 0, # unit: degrees Celcius
            'TempOperationMax': 40, # unit: degrees Celcius
            'PrecipitationMax': 0, # unit: cm # intensity: moderate
            'SnowfallMax': 0 # unit: cm # intensity: light
        }
    }

    droneChoice = 'cumulus'

    #print(UAV[droneChoice]['TempOperationMin'])

    # Initialize and load data - City 1
    # CleanedObservationsOdense, CleanedObservationsRinge, CleanedObservationsSvendborg
    reader_c1 = ibm_weather_csv(
        'DronePlanning/CleanedObservationsOdense.csv',
        'Odense', 2016,
        UAV[droneChoice]['WindSpeedOperationMax'],
        UAV[droneChoice]['WindGustOperationMax'],
        UAV[droneChoice]['TempOperationMin'],
        UAV[droneChoice]['TempOperationMax'],
        UAV[droneChoice]['PrecipitationMax'],
        UAV[droneChoice]['SnowfallMax'],
        debug = True
    )
    reader_c1.loadCSV()

    # Initialize and load data - City 2
    reader_c2 = ibm_weather_csv(
        'DronePlanning/CleanedObservationsSvendborg.csv',
        'Svendborg', 2016,
        UAV[droneChoice]['WindSpeedOperationMax'],
        UAV[droneChoice]['WindGustOperationMax'],
        UAV[droneChoice]['TempOperationMin'],
        UAV[droneChoice]['TempOperationMax'],
        UAV[droneChoice]['PrecipitationMax'],
        UAV[droneChoice]['SnowfallMax'],
        debug = True
    )
    reader_c2.loadCSV()


    # Output to static HTML file
    output_file("webpages/output.html", title="Drone planning using weather data")


    # %%%%%%%%%%%%%%%%%% WIND VISUALIZATION %%%%%%%%%%%%%%%%%%

    # %%%%%%%%% time plot of WIND SPEED and GUST - START %%%%%%%%%
    # create a new plot
    p1 = reader_c1.createTimePlot('Wind', 'Date and time', 'Wind speed [m/s]')
    # Plot content
    p1.line(reader_c1.DateSGMT, reader_c1.WindGustSurfaceMpsS, legend="Wind gusts - %s" % reader_c1.city, alpha=0.8, color="green")
    p1.line(reader_c1.DateSGMT, reader_c1.WindSpeedMpsS, legend="Wind speed - %s" % reader_c1.city, alpha=0.8)
    p1.line([reader_c1.DateSGMT[0], reader_c1.DateSGMT[-1]], [reader_c1.WindSpeedOperationMax, reader_c1.WindSpeedOperationMax], legend="Wind speed limit = %0d m/s" % reader_c1.WindSpeedOperationMax, line_color="red", line_dash="2 4")
    p1.line([reader_c1.DateSGMT[0], reader_c1.DateSGMT[-1]], [reader_c1.WindGustOperationMax, reader_c1.WindGustOperationMax], legend="Gust speed limit = %0d m/s" % reader_c1.WindGustOperationMax, line_color="red", line_dash="4 2")
    # %%%%%%%%% time plot of WIND SPEED and GUST - END %%%%%%%%%

    # %%%%%%%%% histogram of WIND SPEED - START %%%%%%%%%
    p8 = figure(title="Wind speed",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(reader_c1.WindSpeedMpsS,bins=20)
    p8.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    # Add labels
    p8.xaxis.axis_label = "Wind speed [m/s] - %s" % reader_c1.city
    p8.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of WIND SPEED - END %%%%%%%%%

    # %%%%%%%%% histogram of WIND GUST - START %%%%%%%%%
    p9 = figure(title="Wind gusts",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(reader_c1.WindGustSurfaceMpsS,bins=20)
    p9.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    # Add labels
    p9.xaxis.axis_label = "Wind gusts [m/s] - %s" % reader_c1.city
    p9.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of WIND GUST - END %%%%%%%%%


    # %%%%%%%%%%%%%%%%%% TEMPERATURE VISUALIZATION %%%%%%%%%%%%%%%%%%

    # %%%%%%%%% time plot of TEMPERATURE (different types) - START %%%%%%%%%
    # create a new plot
    p2 = reader_c1.createTimePlot('Temperature', 'Date and time', 'Temperature [°C]')
    # Plot content
    p2.line(reader_c1.DateSGMT, reader_c1.TempSurfaceCS, legend="Temperature - %s" % reader_c1.city, alpha=0.8)
    #p2.line(reader_c1.DateSGMT, reader_c1.TempWindChillCS, legend="Wind Chill Temperature - %s" % reader_c1.city, alpha=0.8, color="green")
    #p2.line(reader_c1.DateSGMT, reader_c1.TempApparentCS, legend="Apparent Temperature - %s" % reader_c1.city, alpha=0.8, color="orange")
    p2.line(reader_c1.DateSGMT, reader_c1.TempSurfaceDewpointCS, legend="Dewpoint Temperature - %s" % reader_c1.city, alpha=0.8, color="green")
    # Draw illustrative lines
    p2.line([reader_c1.DateSGMT[0], reader_c1.DateSGMT[-1]], [reader_c1.TempOperationMin, reader_c1.TempOperationMin], legend="Temperature min = %0d °C" % reader_c1.TempOperationMin, line_color="red", line_dash="2 4")
    p2.line([reader_c1.DateSGMT[0], reader_c1.DateSGMT[-1]], [reader_c1.TempOperationMax, reader_c1.TempOperationMax], legend="Temperature max = %0d °C" % reader_c1.TempOperationMax, line_color="red", line_dash="4 2")
    # %%%%%%%%% time plot of TEMPERATURE (different types) - END %%%%%%%%%

    # %%%%%%%%% histogram of TEMPERATURE - START %%%%%%%%%
    p10 = figure(title="Temperature",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(reader_c1.TempSurfaceCS,bins=20)
    p10.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    # Add labels
    p10.xaxis.axis_label = "Temperature [°C] - %s" % reader_c1.city
    p10.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of TEMPERATURE - END %%%%%%%%%

    # %%%%%%%%% histogram of Apparent TEMPERATURE - START %%%%%%%%%
    p11 = figure(title="Apparent Temperature",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(reader_c1.TempApparentCS,bins=20)
    p11.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    # Add labels
    p11.xaxis.axis_label = "Apparent Temperature [°C] - %s" % reader_c1.city
    p11.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of Apparent TEMPERATURE - END %%%%%%%%%


    # %%%%%%%%%%%%%%%%%% PRECIPITATION VISUALIZATION %%%%%%%%%%%%%%%%%%

    # %%%%%%%%% time plot of PRECIPITATION - START %%%%%%%%%
    # create a new plot
    p3 = reader_c1.createTimePlot('Precipitation', 'Date and time', 'Precipitation [cm]')
    p3.line(reader_c1.DateSGMT, reader_c1.PrecipitationPreviousHourCmS, legend="Precipitation - %s" % reader_c1.city, alpha=0.8)
    # %%%%%%%%% time plot of PRECIPITATION - END %%%%%%%%%

    # %%%%%%%%% time plot of PRECIPITATION:SNOWFALL - START %%%%%%%%%
    # create a new plot
    p4 = reader_c1.createTimePlot('Snowfall', 'Date and time', 'Snowfall [cm]')
    p4.line(reader_c1.DateSGMT, reader_c1.SnowfallCmS, legend="Snowfall - %s" % reader_c1.city, alpha=0.8)
    # %%%%%%%%% time plot of PRECIPITATION:SNOWFALL - END %%%%%%%%%


    # %%%%%%%%%%%%%%%%%% DATA ANALYSIS %%%%%%%%%%%%%%%%%%

    # %%%%%%%%% Analysis WIND %%%%%%%%%
    periods_wind, hoursOfWind = reader_c1.analyseWind()
    #print periods_wind

    # %%%%%%%%% histogram of Consequitive WIND hours - START %%%%%%%%%
    p5 = figure(title="Wind analysis",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(periods_wind,bins=30)
    p5.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p5.xaxis.axis_label = 'Consequitive hours with wind velocity > %d m/s' % reader_c1.WindSpeedOperationMax
    p5.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of Consequitive WIND hours - END %%%%%%%%%

    # %%%%%%%%% WIND analysis plot - START %%%%%%%%%
    # Explanation: it shows how many days there have been ex. 4 hours of wind exceding the conditions. In other words if there is occurences of hours with wind above conditions for more than 24 hours the whole day is unflyable.
    p7 = figure(
        tools="pan,box_zoom,reset,save,hover",
        title="Wind analysis: days with wind exceeding conditions, divided in time interval_c1s",
        x_axis_label='Hours with wind velocity > %d m/s (capped below 2 hours)' % reader_c1.WindSpeedOperationMax,
        y_axis_label='Days',
        plot_height = reader_c1.plotHeight, plot_width = reader_c1.plotWidth
    )
    p7.line(range(25), hoursOfWind, alpha=0.6)
    p7.circle(range(25), hoursOfWind, size=10, alpha=0.8)
    # %%%%%%%%% WIND analysis plot - END %%%%%%%%%

    # %%%%%%%%% Analysis TEMPERATURE %%%%%%%%%
    periods_temperature = reader_c1.analyseTemperature()
    #print periods_temperature

    # %%%%%%%%% histogram of Consequitive TEMPERATURE hours - START %%%%%%%%%
    # p6 = figure(title="Temperature analysis - using apparent temperature",tools="save",plot_width=reader_c1.plotWidth/2,plot_height=reader_c1.plotHeight)
    p6 = figure(title="Temperature analysis",tools="save",plot_width=floor(reader_c1.plotWidth/2),plot_height=reader_c1.plotHeight)
    hist,bins=np.histogram(periods_temperature,bins=30)
    p6.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p6.xaxis.axis_label = 'Consequitive hours with temperature < %d °C' % reader_c1.TempOperationMin
    p6.yaxis.axis_label = 'Occurences'
    # %%%%%%%%% histogram of Consequitive TEMPERATURE hours - START %%%%%%%%%


    # %%%%%%%%% Analysis COMBINED - NOTE City 1 %%%%%%%%%
    periods_combined_c1, hoursExcedingConditions_c1, results_combined_arr_c1  = reader_c1.analyseCombined()

    # Find the number that occurs the most in the data
    most_occurance = max(periods_combined_c1,key=periods_combined_c1.count)
    occurance_no = periods_combined_c1.count(most_occurance)
    max_x = ceil(max(periods_combined_c1))
    min_x = floor(min(periods_combined_c1))
    data_points = len(periods_combined_c1)
    # print(most_occurance)
    # print(occurance_no)
    # print(max_x)
    # print(min_x)
    # %%%%%%%%% histogram of Consequitive COMBINED hours - START %%%%%%%%%
    hist,bins=np.histogram(periods_combined_c1) #,bins=10
    # print(bins)
    # print(hist)
    p12 = figure(tools="save",plot_width=reader_c1.plotHeight,plot_height=reader_c1.plotHeight, y_range=(0,floor(max(hist)*1.15)), x_range=(min_x-1, max_x+1)) # title="Combined analysis",
    p12.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color=RGB(0,0,0), color=RGB(116,173,209))
    p12.xaxis.axis_label = 'Consequitive hours exceeding conditions'
    p12.yaxis.axis_label = 'Occurences'
    #latex_plot_font(p12)
    #export_plot__png_eps_pdf(p12, 'hist_'+reader_c1.city) # %%%%%%%%% Save plot p14 as svg and pdf %%%%%%%%%



    # %%%%%%%%% histogram of Consequitive COMBINED hours - END %%%%%%%%%

    #sys.exit(1)

    # %%%%%%%%% COMBINED analysis plot - START %%%%%%%%%
    p13 = figure(
        tools="pan,box_zoom,reset,save,hover",
        title="Combined analysis: days with conditions exceeding limits, divided in time intervals",
        x_axis_label='Hours exceeding conditions (capped below 2 hours)',
        y_axis_label='Days',
        plot_height = reader_c1.plotHeight, plot_width = reader_c1.plotWidth
    )
    p13.line(range(25), hoursExcedingConditions_c1, alpha=0.6)
    p13.circle(range(25), hoursExcedingConditions_c1, size=10, alpha=0.8)
    # %%%%%%%%% COMBINED analysis plot - END %%%%%%%%%

    # %%%%%%%%% Illustrative COMBINED analysis plot - START %%%%%%%%%
    start_time_c1 = reader_c1.DateSGMT[0].strftime('%m/%d/%Y')
    #print start_time
    rng_c1 = pd.date_range(start=start_time_c1, periods=reader_c1.days, freq='D' ) # reader_c1.days+1
    #print rng_c1
    interval_c1 = pd.DataFrame({'start' : rng_c1 })
    interval_c1['end'] = interval_c1['start'] + pd.Timedelta(hours=24)#pd.Timedelta(hours=23,minutes=59,seconds=59)
    #print interval_c1,"\n\n"

    p14 = figure(x_axis_type='datetime', plot_height=floor(reader_c1.plotHeight/3*2),plot_width=reader_c1.plotWidth, tools="box_zoom,reset,save", x_range=(interval_c1['start'][0], interval_c1['end'][reader_c1.days-1]), y_range=(0, 24)) # the plot format/size is set by heigh and width and the sizing_mode makes it reponsive
    draw_quads(p14, results_combined_arr_c1, interval_c1)
    change_plot_axis_names(p14, "Date [m/Y]", "Time of day [H:M]")
    latex_plot_font(p14)
    p14.xaxis[0].ticker.desired_num_ticks = 12
    p14.yaxis[0].ticker.desired_num_ticks = 12
    p14.yaxis[0].ticker.num_minor_ticks = 3
    p14.yaxis.major_label_overrides = {0:'00:00', 0.5:'00:30', 1:'01:00', 1.5:'01:30', 2:'02:00', 2.5:'02:30', 3:'03:00', 3.5:'03:30', 4:'04:00', 4.5:'04:30', 5:'05:00', 5.5:'05:30', 6:'06:00', 6.5:'06:30', 7:'07:00', 7.5:'07:30', 8:'08:00', 8.5:'08:30', 9:'09:00', 9.5:'09:30', 10:'10:00', 10.5:'10:30', 11:'11:00', 11.5:'11:30', 12:'12:00', 12.5:'12:30', 13:'13:00', 13.5:'13:30', 14:'14:00', 14.5:'14:30', 15:'15:00', 15.5:'15:30', 16:'16:00', 16.5:'16:30', 17:'17:00', 17.5:'17:30', 18:'18:00', 18.5:'18:30', 19:'19:00', 19.5:'19:30', 20:'20:00', 20.5:'20:30', 21:'21:00', 21.5:'21:30', 22:'22:00', 22.5:'22:30', 23:'23:00', 23.5:'23:30', 24:'00:00'}
    p14.yaxis.minor_tick_line_color = None
    # Add lines to illustrate daylight. The times are based on the spring mean day https://www.dmi.dk/nyheder/arkiv/nyheder-2012/daglaengden-dykker-under-ti-timer/
    p14.line([interval_c1['start'][0], interval_c1['end'][reader_c1.days-1]], [8.15-1, 8.15-1],   line_color="black", line_dash="2 4", legend="Average sunrise and sunset")
    p14.line([interval_c1['start'][0], interval_c1['end'][reader_c1.days-1]], [20.15-1, 20.15-1], line_color="black", line_dash="2 4")
    custom_legend(p14) # Format legend
    # %%%%%%%%% Illustrative COMBINED analysis plot - END %%%%%%%%%
    export_plot__png_eps_pdf(p14, reader_c1.city) # %%%%%%%%% Save plot p14 as svg and pdf %%%%%%%%%

    # %%%%%%%%% Analysis COMBINED - NOTE City 2%%%%%%%%%
    periods_combined_c2, hoursExcedingConditions_c2, results_combined_arr_c2  = reader_c2.analyseCombined()
    # %%%%%%%%% Copy from above %%%%%%%%%
    # %%%%%%%%% Illustrative COMBINED analysis plot - START %%%%%%%%%
    start_time_c2 = reader_c2.DateSGMT[0].strftime('%m/%d/%Y')
    #print start_time
    rng_c2 = pd.date_range(start=start_time_c2, periods=reader_c2.days, freq='D' ) # reader_c2.days+1
    #print rng_c2
    interval_c2 = pd.DataFrame({'start' : rng_c2 })
    interval_c2['end'] = interval_c2['start'] + pd.Timedelta(hours=24)#pd.Timedelta(hours=23,minutes=59,seconds=59)
    #print interval_c2,"\n\n"

    p15 = figure(x_axis_type='datetime', plot_height=floor(reader_c2.plotHeight/3*2),plot_width=reader_c2.plotWidth, tools="box_zoom,reset,save", x_range=(interval_c2['start'][0], interval_c2['end'][reader_c2.days-1]), y_range=(0, 24)) # the plot format/size is set by heigh and width and the sizing_mode makes it reponsive
    draw_quads(p15, results_combined_arr_c2, interval_c2)
    change_plot_axis_names(p15, "Date [m/Y]", "Time of day [H:M]")
    latex_plot_font(p15)
    p15.xaxis[0].ticker.desired_num_ticks = 12
    p15.yaxis[0].ticker.desired_num_ticks = 12
    p15.yaxis[0].ticker.num_minor_ticks = 3
    p15.yaxis.major_label_overrides = {0:'00:00', 0.5:'00:30', 1:'01:00', 1.5:'01:30', 2:'02:00', 2.5:'02:30', 3:'03:00', 3.5:'03:30', 4:'04:00', 4.5:'04:30', 5:'05:00', 5.5:'05:30', 6:'06:00', 6.5:'06:30', 7:'07:00', 7.5:'07:30', 8:'08:00', 8.5:'08:30', 9:'09:00', 9.5:'09:30', 10:'10:00', 10.5:'10:30', 11:'11:00', 11.5:'11:30', 12:'12:00', 12.5:'12:30', 13:'13:00', 13.5:'13:30', 14:'14:00', 14.5:'14:30', 15:'15:00', 15.5:'15:30', 16:'16:00', 16.5:'16:30', 17:'17:00', 17.5:'17:30', 18:'18:00', 18.5:'18:30', 19:'19:00', 19.5:'19:30', 20:'20:00', 20.5:'20:30', 21:'21:00', 21.5:'21:30', 22:'22:00', 22.5:'22:30', 23:'23:00', 23.5:'23:30', 24:'00:00'}
    p15.yaxis.minor_tick_line_color = None
    # Add lines to illustrate daylight. The times are based on the spring mean day https://www.dmi.dk/nyheder/arkiv/nyheder-2012/daglaengden-dykker-under-ti-timer/
    p15.line([interval_c2['start'][0], interval_c2['end'][reader_c2.days-1]], [8.15-1, 8.15-1],   line_color="black", line_dash="2 4", legend="Average sunrise and sunset")
    p15.line([interval_c2['start'][0], interval_c2['end'][reader_c2.days-1]], [20.15-1, 20.15-1], line_color="black", line_dash="2 4")
    custom_legend(p15) # Format legend
    # %%%%%%%%% Illustrative COMBINED analysis plot - END %%%%%%%%%
    export_plot__png_eps_pdf(p15, reader_c2.city) # %%%%%%%%% Save plot p14 as svg and pdf %%%%%%%%%

    # %%%%%%%%% Analysis COMBINED - NOTE Both cities %%%%%%%%%
    periods_combined_c1c2, hoursExcedingConditions_c1c2, results_combined_arr_c1c2 = analyseCombined_2cities(reader_c1, reader_c2)
    # %%%%%%%%% Copy from above %%%%%%%%%
    # %%%%%%%%% Illustrative COMBINED analysis plot - START %%%%%%%%%
    start_time_c1c2 = reader_c1.DateSGMT[0].strftime('%m/%d/%Y')
    #print start_time
    rng_c1c2 = pd.date_range(start=start_time_c1c2, periods=reader_c1.days, freq='D' ) # reader_c1.days+1
    #print rng_c1c2
    interval_c1c2 = pd.DataFrame({'start' : rng_c1c2 })
    interval_c1c2['end'] = interval_c1c2['start'] + pd.Timedelta(hours=24)#pd.Timedelta(hours=23,minutes=59,seconds=59)
    #print(interval_c1c2)

    p16 = figure(x_axis_type='datetime', plot_height=floor(reader_c1.plotHeight/3*2),plot_width=reader_c1.plotWidth, tools="box_zoom,reset,save", x_range=(interval_c1c2['start'][0], interval_c1c2['end'][reader_c1.days-1]), y_range=(0, 24)) # the plot format/size is set by heigh and width and the sizing_mode makes it reponsive
    # formatting
    draw_quads(p16, results_combined_arr_c1c2, interval_c1c2)
    change_plot_axis_names(p16, "Date [m/Y]", "Time of day [H:M]")
    latex_plot_font(p16)
    p16.xaxis[0].ticker.desired_num_ticks = 12
    p16.yaxis[0].ticker.desired_num_ticks = 12
    p16.yaxis[0].ticker.num_minor_ticks = 3
    p16.yaxis.major_label_overrides = {0:'00:00', 0.5:'00:30', 1:'01:00', 1.5:'01:30', 2:'02:00', 2.5:'02:30', 3:'03:00', 3.5:'03:30', 4:'04:00', 4.5:'04:30', 5:'05:00', 5.5:'05:30', 6:'06:00', 6.5:'06:30', 7:'07:00', 7.5:'07:30', 8:'08:00', 8.5:'08:30', 9:'09:00', 9.5:'09:30', 10:'10:00', 10.5:'10:30', 11:'11:00', 11.5:'11:30', 12:'12:00', 12.5:'12:30', 13:'13:00', 13.5:'13:30', 14:'14:00', 14.5:'14:30', 15:'15:00', 15.5:'15:30', 16:'16:00', 16.5:'16:30', 17:'17:00', 17.5:'17:30', 18:'18:00', 18.5:'18:30', 19:'19:00', 19.5:'19:30', 20:'20:00', 20.5:'20:30', 21:'21:00', 21.5:'21:30', 22:'22:00', 22.5:'22:30', 23:'23:00', 23.5:'23:30', 24:'00:00'}
    p16.yaxis.minor_tick_line_color = None
    # Add lines to illustrate daylight. The times are based on the spring mean day https://www.dmi.dk/nyheder/arkiv/nyheder-2012/daglaengden-dykker-under-ti-timer/
    p16.line([interval_c1c2['start'][0], interval_c1c2['end'][reader_c1.days-1]], [8.15-1, 8.15-1],   line_color="black", line_dash="2 4", legend="Average sunrise and sunset")
    p16.line([interval_c1c2['start'][0], interval_c1c2['end'][reader_c1.days-1]], [20.15-1, 20.15-1], line_color="black", line_dash="2 4")
    custom_legend(p16) # Format legend
    # %%%%%%%%% Illustrative COMBINED analysis plot - END %%%%%%%%%
    export_plot__png_eps_pdf(p16, '%s_and_%s' % (reader_c1.city, reader_c2.city)) # %%%%%%%%% Save plot p16 as svg and pdf %%%%%%%%%

    # %%% Histogram
    most_occurance = max(periods_combined_c1c2,key=periods_combined_c1c2.count)
    occurance_no = periods_combined_c1c2.count(most_occurance)
    max_x = ceil(max(periods_combined_c1c2))
    min_x = floor(min(periods_combined_c1c2))
    data_points = len(periods_combined_c1c2)
    # %%%%%%%%% histogram of Consequitive COMBINED hours - START %%%%%%%%%
    hist,bins=np.histogram(periods_combined_c1c2) #,bins=10
    if droneChoice == 'DJIP4':
        p17 = figure(tools="save",plot_width=reader_c1.plotHeight,plot_height=200, x_range=(min_x-1, max_x+1), y_range=(0,100)) # title="Combined analysis", y_range=(0,floor(max(hist)*1.15)) , y_range=(0,100)
    else:
        p17 = figure(tools="save",plot_width=reader_c1.plotHeight,plot_height=200, x_range=(min_x-1, max_x+1), y_range=(0,floor(max(hist)*1.15))) # title="Combined analysis", y_range=(0,floor(max(hist)*1.15)) , y_range=(0,100)
    p17.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color=RGB(0,0,0), color=RGB(116,173,209))
    p17.xaxis.axis_label = 'Consequitive hours outside operational conditions'
    p17.yaxis.axis_label = 'Frequency'#'Occurences'
    if droneChoice == 'DJIP4':
        citation = Label(x=bins[0]+1, y=100-16, x_units='data', y_units='data',
                     text=str(hist[0]), render_mode='canvas',
                     border_line_color='black', border_line_alpha=0.0,
                     background_fill_color='white', background_fill_alpha=0.0, text_font_size="10pt", text_font = 'cmr10', text_color=RGB(255,255,255))

        p17.add_layout(citation)
        # p17.add_layout(Arrow(end=OpenHead(line_color=RGB(255,255,255), line_width=2, size=6), line_color=RGB(255,255,255), x_start=bins[0]+4, y_start=90, x_end=bins[0]+4, y_end=98))
        p17.line([bins[0]+4, bins[0]+4], [90, 98], line_color=RGB(255,255,255))
        p17.line([bins[0]+4, bins[0]+3], [98, 94], line_color=RGB(255,255,255))
        p17.line([bins[0]+4, bins[0]+5], [98, 94], line_color=RGB(255,255,255))
    latex_plot_font(p17)
    export_plot__png_eps_pdf(p17, 'hist_'+reader_c1.city+'_and_'+reader_c2.city) # %%%%%%%%% Save plot p14 as svg and pdf %%%%%%%%%

    sys.exit(0)

    # %%%%%%%%%%%%%%%%%% Bokeh layout building %%%%%%%%%%%%%%%%%%

    # Make legends clickable
    p1.legend.click_policy = "hide"
    p2.legend.click_policy = "hide"
    p3.legend.click_policy = "hide"
    p4.legend.click_policy = "hide"
    p7.legend.click_policy = "hide"

    # %%%%%%%%% TEXT elements - START %%%%%%%%%
    # divTemplate = Div(text="", width = 800)
    divHeader = Div(text="""<center><h1>Drone planning using weather data</h1><br /><h2>Data source 1: IBM, %s, %0d</h2><h2>Data source 2: IBM, %s, %0d</h2><p><i>Data visualzation and analysis by <a href="https://github.com/TobiasLundby" target="_blank">Tobias Lundby</a>, 2019</i></p></center>""" % (reader_c1.city, reader_c1.year, reader_c2.city, reader_c2.year)) # , width=200, height=100
    divVisualization_c1 = Div(text="""<h2>Visualization %s</h2>""" % (reader_c1.city))
    divWind = Div(text="""<h3>Wind</h3>""")
    divTemp = Div(text="""<h3>Temperature</h3>""")
    divPrecipitation = Div(text="""<h3>Precipitation</h3>""")
    divIndividual = Div(text="""<h3>Individual analysis</h3>""")
    divCombined = Div(text="""<h3>Combined weather analysis - single city</h3><p>Wind, temperature, icing percipitation, and snowfall</p>""", width = reader_c1.plotWidth)
    divAnalysis = Div(text="""<h2>Data analysis</h2>""")
    divP14Title = Div(text="<h3>Illustrative weather analysis - combined wind, temperature, icing percipitation, and snowfall</h3>")
    divExplanationP14 = Div(text="""<p><center>transparent = flying OK<br>red = flying not OK; wind, icing, precipitation, snowfall, and temperature exceeding limits<br>orange = flying disencouraged, wind exceeding limit<br>yellow = flying disencouraged, icing risk<br>light blue = flying disencouraged, rain exceeding limit<br>blue = flying disencouraged, snowfall exceeding limit<br>dark blue = flying disencouraged, temperature exceeding limit<br><i>The dashed black lines represents the avg spring daytime GMT (07:09-19:09); source dmi.dk</i></center></p>""")
    divVisualization_c2 = Div(text="""<h2>Visualization %s</h2>""" % (reader_c2.city))
    divP15Title = Div(text="<h3>Illustrative weather analysis - combined wind, temperature, icing percipitation, and snowfall</h3>")
    divExplanationP15 = Div(text="""<p><center>transparent = flying OK<br>red = flying not OK; wind, icing, precipitation, snowfall, and temperature exceeding limits<br>orange = flying disencouraged, wind exceeding limit<br>yellow = flying disencouraged, icing risk<br>light blue = flying disencouraged, rain exceeding limit<br>blue = flying disencouraged, snowfall exceeding limit<br>dark blue = flying disencouraged, temperature exceeding limit<br><i>The dashed black lines represents the avg spring daytime GMT (07:09-19:09); source dmi.dk</i></center></p>""")
    divVisualization_c1c2 = Div(text="""<h2>Visualization %s and %s</h2>""" % (reader_c1.city, reader_c2.city))
    divP16Title = Div(text="<h3>Illustrative weather analysis - combined wind, temperature, icing percipitation, and snowfall</h3>")
    divExplanationP16 = Div(text="""<p><center>transparent = flying OK<br>red = flying not OK; wind, icing, precipitation, snowfall, and temperature exceeding limits<br>orange = flying disencouraged, wind exceeding limit<br>yellow = flying disencouraged, icing risk<br>light blue = flying disencouraged, rain exceeding limit<br>blue = flying disencouraged, snowfall exceeding limit<br>dark blue = flying disencouraged, temperature exceeding limit<br><i>The dashed black lines represents the avg spring daytime GMT (07:09-19:09); source dmi.dk</i></center></p>""")

    # %%%%%%%%% TEXT elements - START %%%%%%%%%

    # %%%%%%%%% Generate layout %%%%%%%%%
    #p = column(widgetbox(divExplanation),s1,s2)
    p = layout(children=[
            [widgetbox(divHeader)],
            [widgetbox(divVisualization_c1)],
            [widgetbox(divWind)],
            [p1],
            [p8,p9],
            [widgetbox(divTemp)],
            [p2],
            [p10,p11],
            [widgetbox(divPrecipitation)],
            [p3],
            [p4],
            [widgetbox(divAnalysis)],
            [widgetbox(divIndividual)],
            [p5,p6],
            [p7],
            [widgetbox(divCombined)],
            [p12],
            [p13],
            [divP14Title],
            [p14],
            [divExplanationP14],
            [widgetbox(divVisualization_c2)],
            [divP15Title],
            [p15],
            [divExplanationP15],
            [widgetbox(divVisualization_c1c2)],
            [divP16Title],
            [p16],
            [divExplanationP16]
        ], sizing_mode='scale_width') # None for no show and , sizing_mode='stretch_both'
    # p = layout(children=[[p1],
    #         [p14]],
    #         sizing_mode='scale_width') # None for no show and , sizing_mode='stretch_both'

    # show the results
    if SHOW_WEBPAGE:
        show(p)
### Main end
