#!/usr/bin/python
# -*- coding: utf-8 -*-

import json

class qgc_sim_path_writer():
    COMMAND_TAKEOFF = 22
    COMMAND_WAYPOINT = 16
    COMMAND_LAND = 21

    def __init__(self, debug = False, file_id = None):
        self.debug = debug
        self.plan = {}
        self.items = []

        self.home_coords = []

        self.home_altitude_rel = 0
        if file_id == None:
            self.file_id = ""
        else:
            self.file_id = file_id

        self.id = 1
        self.frame = 2

        self.add_init()

    def add_init(self):
        self.plan['firmwareType'] = 3
        self.plan['groundStation'] = 'QGroundControl'

    def add_home(self, location):
        self.home_coords = location
        self.home_altitude_rel = location[2]

    def add_plan_waypoint(self, location, command_type = 22, param0 = 0, param1 = 0, param2 = 0, param3 = 0):
        item = {}
        item['autoContinue'] = True
        item['command'] = command_type
        item['coordinate'] = [location[0],location[1],location[2]]
        item['doJumpId'] = self.id
        item['frame'] = self.frame
        item['params'] = [param0,param1,param2,param3]
        item['type'] = 'SimpleItem'
        self.items.append (item)
        self.id += 1

    def add_takeoff(self, location, home_alt_rel = 0):
        self.add_plan_waypoint(location, command_type = self.COMMAND_TAKEOFF)
        self.frame += 1
    def add_waypoint(self, location):
        if len(self.items) >= 1:
            self.add_plan_waypoint(location, command_type = self.COMMAND_WAYPOINT)
        else:
            print 'Please add start location first'
    def add_land(self, location, abort_alt_rel = 0):
        if len(self.items) >= 1:
            #self.frame += 1
            self.add_plan_waypoint(location, command_type = self.COMMAND_LAND, param0 = abort_alt_rel)
            self.frame += 1
        else:
            print 'Please add start location first'

    def write_plan(self):
        if len(self.home_coords) > 0:
            if len(self.items) > 1:
                self.plan['items'] = self.items

                self.plan['plannedHomePosition'] = [self.home_coords[0], self.home_coords[1], self.home_coords[2]]
                self.plan['version'] = 2

                plan_json = json.dumps(self.plan, indent=4, sort_keys=True)
                if self.file_id == '':
                    file_path = open('path.mission','w')
                else:
                    file_path = open('path_%s.mission' % self.file_id,'w')
                file_path.write(plan_json)
                file_path.close()

                if self.file_id == '':
                    file_script = open('gazebo_setup.sh','w')
                else:
                    file_script = open('gazebo_setup_%s.sh' % self.file_id,'w')
                file_script.write('PX4_HOME_LAT=%.05f\n' % self.home_coords[0])
                file_script.write('PX4_HOME_LON=%.05f\n' % self.home_coords[1])
                file_script.write('PX4_HOME_ALT=%.05f' % self.home_altitude_rel)
                file_script.close()
            else:
                print 'Not enough waypoints added'
        else:
            print 'No home location provided'


if __name__ == '__main__':
    path_qgc_writer = qgc_sim_path_writer(True)

    home_loc = [55.4713, 10.3256, 0]
    path_qgc_writer.add_home(home_loc)

    test_loc = [55.4713, 10.3256, 20]
    path_qgc_writer.add_takeoff(test_loc)

    test_loc = [55.4713271,10.32561629,20]
    path_qgc_writer.add_waypoint(test_loc)

    test_loc = [55.4714042,10.3256544,20]
    path_qgc_writer.add_waypoint(test_loc)

    test_loc = [55.4710709,10.3246252,20]
    path_qgc_writer.add_waypoint(test_loc)

    test_loc = [55.4708023,10.3263024,20]
    path_qgc_writer.add_land(test_loc, abort_alt_rel = -5)

    path_qgc_writer.write_plan()
