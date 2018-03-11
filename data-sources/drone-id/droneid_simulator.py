#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: KJ = Kjeld Jensen (kjen@mmmi.sdu.dk), TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-03-?? KJ Example code
2018-03-09 TL Fork of KJ example code
"""

"""
Description:

License: BSD 3-Clause
"""

#print 'Importing libraries'
import requests
#print 'Import done\n'

class droneid_simulator():
	def __init__(self,debug):
		self.debug = debug
	def send_log_entry(self, log):
		r = requests.post("https://droneid.dk/tobias/droneid_log.php", data={'aid': log['aid'], 'lat': log['lat'], 'lon': log['lng'], 'alt': log['alt']})
		#print(r.status_code, r.reason)
		#print(r.text)
		if self.debug:
			if r.status_code == 200 and r.reason == 'OK':
				print "Added simulated drone with id", log['aid'], '- lat:', log['lat'], '- lng:', log['lng'], '- altitude', log['alt']

if __name__ == '__main__':
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
	droneid_simulator_module = droneid_simulator(True)
	droneid_simulator_module.send_log_entry (LOG_TEST_DATA[0])
	droneid_simulator_module.send_log_entry (LOG_TEST_DATA[1])
