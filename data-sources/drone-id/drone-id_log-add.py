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

print 'Importing libraries'
import requests
print 'Import done\n'

def send_log_entry(log):
	r = requests.post("https://droneid.dk/tobias/droneid_log.php", data={'aid': log['aid'], 'lat': log['lat'], 'lon': log['lon'], 'alt': log['alt']})
	print(r.status_code, r.reason)
	print(r.text)

LOG_TEST_DATA = [
	{
		'aid': 900,
		'lat': 55.395,
		'lon': 10.371,
		'alt': 50,
	},
]

send_log_entry (LOG_TEST_DATA[0])
