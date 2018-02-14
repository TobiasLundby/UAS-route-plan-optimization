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
import csv
import numpy as np
from bokeh.io import output_file, show
from bokeh.layouts import gridplot, column, widgetbox, layout
from bokeh.plotting import figure
from bokeh.models.widgets import Div
from datetime import datetime
### Import end

### Define start
MaxOperationWindSpeed_def = 12
MaxOperationWindGusts_def = 15
OperationMinTemperature_def = -10
OperationMaxTemperature_def = 40
use_apparent_temp = True # for the condition checking
### Define end

### Class start
class ibm_weather_csv():
    def __init__(self, inFileName, inCity, inYear, inMaxOperationWindSpeed, inMaxOperationWindGusts, inOperationMinTemperature, inOperationMaxTemperature):
        self.fileName = inFileName;
        self.city = inCity
        self.year = inYear
        self.maxOperationWindSpeed = inMaxOperationWindSpeed
        self.maxOperationWindGusts = inMaxOperationWindGusts
        self.minOperationTemperature = inOperationMinTemperature
        self.maxOperationTemperature = inOperationMaxTemperature

        self.DateSGMT = []

        self.WindSpeedKphS = []
        self.SurfaceWindGustsKphS = []

        self.WindSpeedMpsS = []
        self.SurfaceWindGustsMpsS = []

        self.SurfaceTempCS = []
        self.ApparentTemperatureCS = []
        self.WindChillTemperatureCS = []
        self.SurfaceDewpointTemperatureCS = []

        self.PrecipitationPreviousHourCmS = []
        self.SnowfallCmS = []

        self.samples = 0

        self.plotWidth = 800
        self.plotHeight = 400

        self.debugText = True

        self.days = 0

    def reset(self):
        self.fileName = '';
        self.city = ''
        self.year = np.nan
        self.maxOperationWindSpeed = np.nan
        self.minOperationTemperature = np.nan

        self.DateSGMT = []

        self.WindSpeedKphS = []
        self.SurfaceWindGustsKphS = []

        self.SurfaceTempCS = []
        self.ApparentTemperatureCS = []
        self.WindChillTemperatureCS = []
        self.SurfaceDewpointTemperatureCS = []

        self.PrecipitationPreviousHourCmS = []
        self.SnowfallCmS = []

        self.samples = 0

        self.days = 0

    def loadCSV(self):
        if self.debugText:
            print 'Attempting to open data file'
        with open(self.fileName) as csvfile:
        #with open('DronePlanning/sec.csv') as csvfile:
            if self.debugText:
                print 'Data file opened, attempting data load'
            readCSV = csv.DictReader(csvfile, delimiter=',')
            for row in readCSV:
                # Date load - format 01/01/2016 00:00:00
                DateGMT = datetime.strptime(row['DateHrGmt'], '%m/%d/%Y %H:%M:%S')
                self.DateSGMT.append(DateGMT)

                # Wind speed load
                WindSpeedKph = float(row['WindSpeedKph'])
                self.WindSpeedKphS.append(WindSpeedKph)

                SurfaceWindGustsKph = float(row['SurfaceWindGustsKph'])
                self.SurfaceWindGustsKphS.append(SurfaceWindGustsKph)

                # Temperature load
                SurfaceTempC = float(row['SurfaceTemperatureCelsius'])
                self.SurfaceTempCS.append(SurfaceTempC)

                ApparentTemperatureC = float(row['ApparentTemperatureCelsius'])
                self.ApparentTemperatureCS.append(ApparentTemperatureC)

                WindChillTemperatureC = float(row['WindChillTemperatureCelsius'])
                self.WindChillTemperatureCS.append(WindChillTemperatureC)

                #SurfaceDewpointTemperatureCelsius
                SurfaceDewpointTemperatureC = float(row['SurfaceDewpointTemperatureCelsius'])
                self.SurfaceDewpointTemperatureCS.append(SurfaceDewpointTemperatureC)

                PrecipitationPreviousHourCm = float(row['PrecipitationPreviousHourCentimeters'])
                self.PrecipitationPreviousHourCmS.append(PrecipitationPreviousHourCm)

                SnowfallCm = float(row['SnowfallCentimeters'])
                self.SnowfallCmS.append(SnowfallCm)
            if self.debugText:
                print 'Data loaded'

        #x = np.arange(len(SurfaceTempS))

        self.samples = len(self.DateSGMT)
        self.days = int(round(len(self.DateSGMT)/24))
        if self.debugText:
            print 'Samples:', str(self.samples)
            print 'Days:', str(self.days)


        # convert to the more used m/s
        self.WindSpeedMpsS = np.divide(self.WindSpeedKphS, 3.6)
        self.SurfaceWindGustsMpsS = np.divide(self.SurfaceWindGustsKphS, 3.6)
        #WindSpeedMpsS = [x / 3.6 for x in WindSpeedKphS] #note that the list contains floats, use calc above
        #print(WindSpeedMpsS)
    ## Check condition methods
    def check_conditions_wind(self, sample_nr):
        # true = sattisfies wind conditions
        # self.WindSpeedMpsS[i] > self.maxOperationWindSpeed or self.SurfaceWindGustsMpsS[i] > self.maxOperationWindSpeed
        if self.WindSpeedMpsS[sample_nr] > self.maxOperationWindSpeed:
            return False
        if self.SurfaceWindGustsMpsS[sample_nr] > self.maxOperationWindGusts:
            return False
        return True
    def check_conditions_temp(self, sample_nr):
        # true = sattisfies temp conditions
        if use_apparent_temp:
            if self.ApparentTemperatureCS[sample_nr] < self.minOperationTemperature:
                return False
            if self.ApparentTemperatureCS[sample_nr] > self.maxOperationTemperature:
                return False
            return True
        else:
            if self.SurfaceTempCS[sample_nr] < self.minOperationTemperature:
                return False
            if self.SurfaceTempCS[sample_nr] > self.maxOperationTemperature:
                return False
            return True
    def check_conditions_all(self, sample_nr):
        if self.check_conditions_wind(sample_nr) and self.check_conditions_temp(sample_nr):
            return True
        else:
            return False
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
        # Calculate percentage statistics
        aboveThreshold = 0
        for i in range(self.samples):
        	if self.check_conditions_wind(i) == False:
        		aboveThreshold += 1
                #print self.DateSGMT[i], self.WindSpeedMpsS[i]
        if self.debugText:
            print 'Number of samples above %d m/s = ' % (self.maxOperationWindSpeed), aboveThreshold
            print 'Percentage of samples above %d m/s = ' % (self.maxOperationWindSpeed), aboveThreshold/(self.samples * 1.0)*100.0, '%'

        # Calculate consecutive periods with max conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0

        periods = []
        periodsAbove = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_conditions_wind(i) == False:
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
            print 'Number of periods with reports above %d m/s = ' % (self.maxOperationWindSpeed), len(periods)

            print '0-1 hour : ', per_1h_count
            print '1-2 hours: ', per_2h_count
            print '2-3 hours: ', per_3h_count
            print '3-4 hours: ', per_4h_count
            print "> 4 hours: ",len(periodsAbove)

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
                if self.check_conditions_wind(day*24+sample) == False:
                    extreme_hour_count += 1
            if extreme_hour_count >= 2:
                hoursOfWind[extreme_hour_count] +=1
            elif extreme_hour_count == 0:
                noWindDays += 1
        print hoursOfWind

        return [periods, hoursOfWind]

    def analyseTemperature(self):
        # Calculate percentage statistics
        belowThreshold = 0
        for i in range(self.samples):
        	if self.check_conditions_temp(i) == False:
        		belowThreshold += 1
                #print self.DateSGMT[i], self.ApparentTemperatureCS[i]
        if self.debugText:
            print 'Number of samples below %d °C = ' % (self.minOperationTemperature), belowThreshold
            print 'Percentage of samples below %d °C = ' % (self.minOperationTemperature), belowThreshold/(self.samples * 1.0)*100.0, '%'

        # Calculate consecutive periods with min conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0

        periods = []
        periodsAbove = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_conditions_temp(i) == False:
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
            print 'Number of periods with reports below %d °C = ' % (self.minOperationTemperature), len(periods)

            print '0-1 hour : ', per_1h_count
            print '1-2 hours: ', per_2h_count
            print '2-3 hours: ', per_3h_count
            print '3-4 hours: ', per_4h_count
            print "> 4 hours: ",len(periodsAbove)

        return periods
    def analyseCombined(self):
        # Calculate percentage statistics
        excedingConditions = 0
        for i in range(self.samples):
        	if self.check_conditions_all(i) == False:
        		excedingConditions += 1
                #print self.DateSGMT[i], self.WindSpeedMpsS[i]
        if self.debugText:
            print 'Number of samples exceding conditions = ', excedingConditions
            print 'Percentage of samples exceding conditions = ', excedingConditions/(self.samples * 1.0)*100.0, '%'

        # Calculate consecutive periods with max conditions
        per_1h_count = 0
        per_2h_count = 0
        per_3h_count = 0
        per_4h_count = 0

        periods = []
        periodsAbove = []
        in_period_count = 0

        for i in range(self.samples):
            if self.check_conditions_all(i) == False:
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
            print 'Number of periods with reports exceding conditions = ', len(periods)

            print '0-1 hour : ', per_1h_count
            print '1-2 hours: ', per_2h_count
            print '2-3 hours: ', per_3h_count
            print '3-4 hours: ', per_4h_count
            print "> 4 hours: ",len(periodsAbove)

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
                if self.check_conditions_all(day*24+sample) == False:
                    extreme_hour_count += 1
            if extreme_hour_count >= 2:
                hoursExcedingConditions[extreme_hour_count] +=1
                # if extreme_hour_count == 24:
                #     print "day: ", day
            elif extreme_hour_count == 0:
                withinDays += 1
        print hoursExcedingConditions

        return [periods, hoursExcedingConditions]

### Class end - Main start

if __name__ == '__main__':
    reader = ibm_weather_csv('DronePlanning/CleanedObservationsOdense.csv', 'Odense', 2016, MaxOperationWindSpeed_def, MaxOperationWindGusts_def, OperationMinTemperature_def, OperationMaxTemperature_def)
    reader.loadCSV()


    # output to static HTML file
    output_file("webpages/output.html", title="Drone planning using weather data")

    # create a new plot
    p1 = reader.createTimePlot('Wind', 'Date and time', 'Wind speed [m/s]')

    # Plot content
    #p.line(x, SurfaceTempCS, legend="Surface temperatures", line_dash="4 4")
    #p.circle(x, SurfaceTempCS, legend="Surface temperatures", fill_color="white", size=8)
    p1.line(reader.DateSGMT, reader.SurfaceWindGustsMpsS, legend="Wind gusts - %s" % reader.city, alpha=0.8, color="green")
    p1.line(reader.DateSGMT, reader.WindSpeedMpsS, legend="Wind speed - %s" % reader.city, alpha=0.8)
    p1.line([reader.DateSGMT[0], reader.DateSGMT[-1]], [reader.maxOperationWindSpeed, reader.maxOperationWindSpeed], legend="Wind speed limit = %0d m/s" % reader.maxOperationWindSpeed, line_color="red", line_dash="2 4")
    p1.line([reader.DateSGMT[0], reader.DateSGMT[-1]], [reader.maxOperationWindGusts, reader.maxOperationWindGusts], legend="Gust speed limit = %0d m/s" % reader.maxOperationWindGusts, line_color="red", line_dash="4 2")


    p8 = figure(title="Wind speed",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(reader.WindSpeedMpsS,bins=20)
    p8.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p8.xaxis.axis_label = "Wind speed [m/s] - %s" % reader.city
    p8.yaxis.axis_label = 'Occurences'

    p9 = figure(title="Wind gusts",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(reader.SurfaceWindGustsMpsS,bins=20)
    p9.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p9.xaxis.axis_label = "Wind gusts [m/s] - %s" % reader.city
    p9.yaxis.axis_label = 'Occurences'

    # create a new plot
    p2 = reader.createTimePlot('Temperature', 'Date and time', 'Temperature [°C]')

    # Plot content
    #p.line(x, SurfaceTempCS, legend="Surface temperatures", line_dash="4 4")
    #p.circle(x, SurfaceTempCS, legend="Surface temperatures", fill_color="white", size=8)

    p2.line(reader.DateSGMT, reader.SurfaceTempCS, legend="Temperature - %s" % reader.city, alpha=0.8)
    p2.line(reader.DateSGMT, reader.WindChillTemperatureCS, legend="Wind Chill Temperature - %s" % reader.city, alpha=0.8, color="green")
    p2.line(reader.DateSGMT, reader.ApparentTemperatureCS, legend="Apparent Temperature - %s" % reader.city, alpha=0.8, color="orange")
    p2.line(reader.DateSGMT, reader.SurfaceDewpointTemperatureCS, legend="Dewpoint Temperature - %s" % reader.city, alpha=0.8, color="green")


    p2.line([reader.DateSGMT[0], reader.DateSGMT[-1]], [reader.minOperationTemperature, reader.minOperationTemperature], legend="Temperature min = %0d °C" % reader.minOperationTemperature, line_color="red", line_dash="2 4")
    p2.line([reader.DateSGMT[0], reader.DateSGMT[-1]], [reader.maxOperationTemperature, reader.maxOperationTemperature], legend="Temperature max = %0d °C" % reader.maxOperationTemperature, line_color="red", line_dash="4 2")

    p10 = figure(title="Temperature",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(reader.SurfaceTempCS,bins=20)
    p10.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p10.xaxis.axis_label = "Temperature [°C] - %s" % reader.city
    p10.yaxis.axis_label = 'Occurences'

    p11 = figure(title="Apparent Temperature",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(reader.ApparentTemperatureCS,bins=20)
    p11.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p11.xaxis.axis_label = "Apparent Temperature [°C] - %s" % reader.city
    p11.yaxis.axis_label = 'Occurences'


    # create a new plot
    p3 = reader.createTimePlot('Precipitation', 'Date and time', 'Precipitation [cm]')
    p3.line(reader.DateSGMT, reader.PrecipitationPreviousHourCmS, legend="Precipitation - %s" % reader.city, alpha=0.8)

    # create a new plot
    p4 = reader.createTimePlot('Snowfall', 'Date and time', 'Snowfall [cm]')
    p4.line(reader.DateSGMT, reader.SnowfallCmS, legend="Snowfall - %s" % reader.city, alpha=0.8)

    periods_wind, hoursOfWind = reader.analyseWind()
    #print periods_wind
    p5 = figure(title="Wind analysis",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(periods_wind,bins=30)
    p5.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p5.xaxis.axis_label = 'Consequitive hours with wind velocity > %d m/s' % reader.maxOperationWindSpeed
    p5.yaxis.axis_label = 'Occurences'

    p7 = figure(
        tools="pan,box_zoom,reset,save,hover",
        title="Wind analysis: days with wind exceding conditions, divided in time intervals",
        x_axis_label='Hours with wind velocity > %d m/s (capped below 2 hours)' % reader.maxOperationWindSpeed,
        y_axis_label='Days',
        plot_height = reader.plotHeight, plot_width = reader.plotWidth
    )
    p7.line(range(25), hoursOfWind, alpha=0.6)
    p7.circle(range(25), hoursOfWind, size=10, alpha=0.8)

    periods_temperature = reader.analyseTemperature()
    #print periods_temperature
    p6 = figure(title="Temperature analysis - using apparent temperature",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(periods_temperature,bins=30)
    p6.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p6.xaxis.axis_label = 'Consequitive hours with temperature < %d °C' % reader.minOperationTemperature
    p6.yaxis.axis_label = 'Occurences'

    periods_combined, hoursExcedingConditions = reader.analyseCombined()
    p12 = figure(title="Combined analysis",tools="save",plot_width=reader.plotWidth/2,plot_height=reader.plotHeight)
    hist,bins=np.histogram(periods_combined,bins=30)
    p12.quad(top=hist, bottom=0, left=bins[:-1], right=bins[1:],line_color="blue")
    p12.xaxis.axis_label = 'Consequitive hours exceding conditions'
    p12.yaxis.axis_label = 'Occurences'

    p13 = figure(
        tools="pan,box_zoom,reset,save,hover",
        title="Combined analysis: days with conditions exceding limits, divided in time intervals",
        x_axis_label='Hours exceding conditions (capped below 2 hours)',
        y_axis_label='Days',
        plot_height = reader.plotHeight, plot_width = reader.plotWidth
    )
    p13.line(range(25), hoursExcedingConditions, alpha=0.6)
    p13.circle(range(25), hoursExcedingConditions, size=10, alpha=0.8)

    # Make legends clickable
    p1.legend.click_policy = "hide"
    p2.legend.click_policy = "hide"
    p3.legend.click_policy = "hide"
    p4.legend.click_policy = "hide"
    p7.legend.click_policy = "hide"

    # Text
    divHeader = Div(text="""<center><h1>Drone planning using weather data</h1><br /><h2>Data: IBM, %s, %0d</h2><p><i>Data visualzation and analysis by <a href="https://github.com/TobiasLundby" target="_blank">Tobias Lundby</a>, 2018</i></p></center>""" % (reader.city, reader.year), width = reader.plotWidth) # , width=200, height=100
    divVisualization = Div(text="""<h2>Visualization</h2>""", width = reader.plotWidth) # , width=200, height=100
    divWind = Div(text="""<h3>Wind</h3>""", width = reader.plotWidth) # , width=200, height=100
    divTemp = Div(text="""<h3>Temperature</h3>""", width = reader.plotWidth) # , width=200, height=100
    divPrecipitation = Div(text="""<h3>Precipitation</h3>""", width = reader.plotWidth) # , width=200, height=100
    divIndividual = Div(text="""<h3>Individual analysis</h3>""", width = reader.plotWidth) # , width=200, height=100
    divCombined = Div(text="""<h3>Combined analysis</h3><p>Wind and temperature excluding percipitation and snow</p>""", width = reader.plotWidth) # , width=200, height=100
    divAnalysis = Div(text="""<h2>Data analysis</h2>""", width = reader.plotWidth) # , width=200, height=100

    # Generate layout
    #p = column(widgetbox(divExplanation),s1,s2)
    p = layout([
            [widgetbox(divHeader)],
            [widgetbox(divVisualization)],
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
            [p13]
        ]) # None for no show and , sizing_mode='stretch_both'

        # show the results
    show(p)
### Main end
