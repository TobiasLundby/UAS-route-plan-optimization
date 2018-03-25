#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: KJ = Kjeld Jensen (kjen@mmmi.sdu.dk), TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-03-?? KJ Example code
2018-03-09 TL Fork of KJ example code
2018-03-12 TL Made into class for use in other programs
"""

"""
Description:
Simulator class for the DroneID project where drones manually can be added to the system.
License: BSD 3-Clause
"""

#print 'Importing libraries'
import requests
from termcolor import colored
#print 'Import done\n'

class droneid_simulator():
	aid_limit_low = 900
	aid_limit_high = 909
	def __init__(self,debug = False):
		self.debug = debug
	def send_log_entry(self, log):
		if log['aid'] < self.aid_limit_low or log['aid'] > self.aid_limit_high:
			print colored("ERROR:", 'red'), colored("Aircraft ID outside allowed range. Requested aircraft ID: %i, allowed %i-%i" % (log['aid'], self.aid_limit_low, self.aid_limit_high), 'yellow')
		else:
			try:
				r = requests.post("https://droneid.dk/tobias/droneid_log.php", data={'aid': log['aid'], 'lat': log['lat'], 'lon': log['lng'], 'alt': log['alt']}, timeout=2)
			except requests.exceptions.Timeout:
			    # Maybe set up for a retry, or continue in a retry loop
				print colored("Request has timed out", 'red')
			except requests.exceptions.TooManyRedirects:
			    # Tell the user their URL was bad and try a different one
				print colored("Request has too many redirects", 'red')
			except requests.exceptions.RequestException as e:
				# catastrophic error. bail.
				print colored("Request error", 'red')
				print colored(e, 'yellow')
				# sys.exit(1)
			else:
				#print(r.status_code, r.reason)
				#print(r.text)
				#print "Everything good"
				if self.debug:
					if r.status_code == 200 and r.reason == 'OK':
						print colored( ("Added simulated drone with id", log['aid'], '- lat:', log['lat'], '- lng:', log['lng'], '- altitude', log['alt']) , 'green')

if __name__ == '__main__':
	LOG_TEST_DATA = [
		{
			'aid': 900,
			'lat': 55.399,
			'lng': 10.385,
			'alt': 50,
		},
		{
			'aid': 909,
			'lat': 55.400,
			'lng': 10.386,
			'alt': 100,
		}
	]
	droneid_simulator_module = droneid_simulator(True)
	droneid_simulator_module.send_log_entry (LOG_TEST_DATA[0])
	droneid_simulator_module.send_log_entry (LOG_TEST_DATA[1])
