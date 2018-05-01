#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pykml import parser
import time

class KML_no_fly_zones_parser():
    def __init__(self, file_name):
        self.file_name = file_name

    def parse_file(self):
        with open(self.file_name) as f:
            file = parser.parse(f).getroot()
        self.coordinate3d_combined = []
        itr = 0
        for element in file.Document.Placemark:
            print 'Placemark', itr
            itr += 1
            try:
                element.Polygon
            except AttributeError:
                print 'Something is wrong about the zone formatting; trying different method'
                try:
                    element.MultiGeometry
                except AttributeError:
                    print 'Something is wrong about the zone formatting; skipping zone!'
                    continue
                else:
                    print '\nMultiGeometry object, amount of subpolygons', len(element.MultiGeometry.Polygon)

                    # Save the values that are the same for each subpolygon
                    element_style = str(element.styleUrl)
                    element_style = element_style[9:]
                    element_id = str(element.id)
                    try:
                        element_name = str(element.name)
                    except UnicodeEncodeError:
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
                        print element_id+'-'+str(sub_itr)
                        self.coordinate3d_combined.append({'style': element_style, 'id': element_id+'-'+str(sub_itr), 'name': element_name, 'coordinates': coordinate3d_placemark})
                        sub_itr += 1
            else:
                print '\nSingle Polygon object'
                element_style = str(element.styleUrl)
                element_style = element_style[9:]
                element_id = str(element.id)
                try:
                    element_name = str(element.name)
                except UnicodeEncodeError:
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
                print element_id
                self.coordinate3d_combined.append({'style': element_style, 'id': element_id, 'name': element_name, 'coordinates': coordinate3d_placemark})
        if len(self.coordinate3d_combined) == 0:
            print 'No no-fly zones were parsed'

    def print_zones(self):
        for element in self.coordinate3d_combined:
            self.print_zone(0, element)
    def print_zone(self, zone_no, zone=None):
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
        return self.coordinate3d_combined[zone_no]['coordinates']

    def get_zone_style(self, zone_no):
        return self.coordinate3d_combined[zone_no]['style']

    def get_zone_id(self, zone_no):
        return self.coordinate3d_combined[zone_no]['id']

    def get_zone_name(self, zone_no):
        return self.coordinate3d_combined[zone_no]['name']

    def get_zone(self, zone_no):
        return self.coordinate3d_combined[zone_no]

    def get_zones(self):
        return self.coordinate3d_combined

    def get_number_of_zones(self):
        return len(self.coordinate3d_combined)


if __name__ == '__main__':
    # Run self test
    test = KML_no_fly_zones_parser('KmlUasZones_2018-02-27-18-24.kml')

    print '\n\nParsing KML file'
    test.parse_file()

    print '\n\nPrinting parsed KML no-fl zones'
    test.print_zones()

    print '\n\nPrinting selected zone'
    test.print_zone(0)

    print '\n\nPrinting seleced zone fields from acessor methods'
    print test.get_zone(0)
    print test.get_zone_style(0)
    print test.get_zone_id(0)
    print test.get_zone_name(0)
    print test.get_zone_coordinates(0)

    print '\n\nPrinting the amount of zones; note that this includes the subzones defined by MultiGeometry'
    print test.get_number_of_zones()
