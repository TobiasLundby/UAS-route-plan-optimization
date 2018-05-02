#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: KML parser for the no-fly zones obtained from https://www.droneluftrum.dk/
             The KML parser can handle Document.Placemark.Polygon and Document.Placemark.MultiGeometry.Polygon (generates sub placemarks with a modifies ID)
             The parsing can be verfied by comparing the results to the results on https://www.techgen.dk/msc/kml-viewer.html
    License: BSD 3-Clause
"""

from pykml import parser
import time

class kml_no_fly_zones_parser():
    def __init__(self, file_name, debug = False):
        """
        Init method
        Input: filename along with optional debug parameter which toogles debug messages
        Output: none
        """
        self.file_name = file_name
        self.debug = debug
        self.placemarks = 0

    def parse_file(self):
        """
        Parses the input file given by the already obtained filename; support reload
        Input: none
        Output: bool (False: parsing failed, True: parsing successful)
        """
        with open(self.file_name) as f:
            file = parser.parse(f).getroot()
        self.coordinate3d_combined = []
        itr = 0 # Incremented in the beginning so first element has itr=1
        for element in file.Document.Placemark:
            itr += 1
            if self.debug:
                print '\nPlacemark', itr
            try:
                element.Polygon
            except AttributeError:
                if self.debug:
                    print 'Something is wrong about the zone formatting; trying different method'
                try:
                    element.MultiGeometry
                except AttributeError:
                    if self.debug:
                        print 'Something is wrong about the zone formatting; skipping zone!'
                    continue
                else:
                    if self.debug:
                        print 'MultiGeometry object, amount of subpolygons', len(element.MultiGeometry.Polygon)

                    # Save the values that are the same for each subpolygon
                    element_style = str(element.styleUrl)
                    element_style = element_style[9:]
                    element_id = str(element.id)
                    try:
                        element_name = str(element.name)
                    except UnicodeEncodeError:
                        if self.debug:
                            print 'Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\''
                        element_name = 'NaN'
                    sub_itr = 0
                    for sub_element in element.MultiGeometry.Polygon:
                        coordinates_raw =  sub_element.outerBoundaryIs.LinearRing.coordinates
                        coordinates_str_split = str(coordinates_raw).split(' ')
                        del coordinates_str_split[-1] # delete the last element since it is just 0.0 and not 3 elements

                        coordinate3d_placemark = []
                        for coordinate in coordinates_str_split:
                            coordinate3d = []
                            sub_coordinates = coordinate.split(',')
                            for sub_coordinate in sub_coordinates:
                                coordinate3d.append(float(sub_coordinate))
                            coordinate3d_placemark.append(coordinate3d)
                        if self.debug:
                            print element_id+'-'+str(sub_itr)
                        self.coordinate3d_combined.append({'style': element_style, 'id': element_id+'-'+str(sub_itr), 'name': element_name, 'coordinates': coordinate3d_placemark})
                        sub_itr += 1
            else:
                if self.debug:
                    print 'Single Polygon object'
                element_style = str(element.styleUrl)
                element_style = element_style[9:]
                element_id = str(element.id)
                try:
                    element_name = str(element.name)
                except UnicodeEncodeError:
                    if self.debug:
                        print 'Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\''
                    element_name = 'NaN'

                coordinates_raw =  element.Polygon.outerBoundaryIs.LinearRing.coordinates
                coordinates_str_split = str(coordinates_raw).split(' ')
                del coordinates_str_split[-1] # delete the last element since it is just 0.0 and not 3 elements

                coordinate3d_placemark = []
                for coordinate in coordinates_str_split:
                    coordinate3d = []
                    sub_coordinates = coordinate.split(',')
                    for sub_coordinate in sub_coordinates:
                        coordinate3d.append(float(sub_coordinate))
                    coordinate3d_placemark.append(coordinate3d)
                if self.debug:
                    print element_id
                self.coordinate3d_combined.append({'style': element_style, 'id': element_id, 'name': element_name, 'coordinates': coordinate3d_placemark})
        self.placemarks = itr
        if len(self.coordinate3d_combined) == 0:
            print 'No no-fly zones were parsed'
            return False
        else:
            return True

    def print_zones(self):
        """
        Prints all of the no-fly zones
        Input: none
        Output: none but printing in the terminal
        """
        for element in self.coordinate3d_combined:
            self.print_zone(0, element)
    def print_zone(self, zone_no, zone=None):
        """
        Prints a single no-fly zone
        Input: zone number and optional zone (if provided it uses the provided zone instead of adressing a number)
        Output: none but printing in the terminal
        """
        if zone == None:
            print '\nStyle: %s' % self.coordinate3d_combined[zone_no]['style']
            print 'ID: %s' % self.coordinate3d_combined[zone_no]['id']
            print 'Name: %s' % self.coordinate3d_combined[zone_no]['name']
            for element in self.coordinate3d_combined[zone_no]['coordinates']:
                print '   '+str(element)
        else:
            print '\nStyle: %s' % zone['style']
            print 'ID: %s' % zone['id']
            print 'Name: %s' % zone['name']
            for element in zone['coordinates']:
                print '   '+str(element)

    def get_zone_coordinates(self, zone_no):
        """
        Returns no-fly zone coordinates of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone coordinates
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['coordinates']
        else:
            return []

    def get_zone_style(self, zone_no):
        """
        Returns no-fly zone style of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone style
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['style']
        else:
            return ''

    def get_zone_id(self, zone_no):
        """
        Returns no-fly zone ID of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone ID
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['id']
        else:
            return ''

    def get_zone_name(self, zone_no):
        """
        Returns no-fly zone name of the argument provided no-fly zone (note that if the parsing failed with the name it returns 'NaN')
        Input: zone number
        Output: no-fly zone name
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['name']
        else:
            return []

    def get_zone(self, zone_no):
        """
        Returns a no-fly zone object of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone
        """
        return self.coordinate3d_combined[zone_no]

    def get_zones(self):
        """
        Returns all the no-fly zones
        Input: none
        Output: no-fly zones
        """
        return self.coordinate3d_combined

    def get_number_of_zones(self):
        """
        Returns the number of no-fly zones
        Input: none
        Output: number of no-fly zones (int)
        """
        return len(self.coordinate3d_combined)
    def get_number_of_placemarks(self):
        """
        Returns the number of placemarks parsed
        Input: none
        Output: number of placemarks (int)
        """
        return self.placemarks

if __name__ == '__main__':
    # Run self test
    #test = kml_no_fly_zones_parser('KmlUasZones_2018-02-27-18-24.kml')
    test = kml_no_fly_zones_parser('KmlUasZones_sec2.kml')

    print '\n\nParsing KML file'
    if test.parse_file():
        print 'The KML has successfully been parsed; parsed %i placemarks and %i no-fly zones (includes subzones defined in MultiGeometry objects)' % (test.get_number_of_placemarks(), test.get_number_of_zones())

        print '\n\nPrinting parsed KML no-fl zones in 2s'
        time.sleep(2)
        test.print_zones()

        print '\n\nPrinting selected zone in 2s'
        time.sleep(2)
        test.print_zone(0)

        print '\n\nPrinting seleced zone fields from acessor methods in 2s'
        time.sleep(2)
        print test.get_zone(0)
        print test.get_zone_style(0)
        print test.get_zone_id(0)
        print test.get_zone_name(0)
        print test.get_zone_coordinates(0)
