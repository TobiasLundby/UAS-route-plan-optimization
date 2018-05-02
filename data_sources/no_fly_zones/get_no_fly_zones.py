#!/usr/bin/env python
"""
2018-01-05 TL First version
"""

"""
Description:
This scripts downloads the No-Fly zones from droneluftrum by first acquirring an
access token and then using this for the actual download.
Library 'requests' installation, see 'README_requests.md'
License: BSD 3-Clause
"""

#p rint 'Importing libraries'
import requests
from requests.auth import HTTPBasicAuth
import json
from urllib2 import urlopen, URLError, HTTPError
import time # wait
import datetime # datetime.now
import sys # exit (use quit() when you only want to terminate spawning script, not all)
# print 'Import done'

class get_no_fly_zones():
	print_raw_response = False

	def __init__(self, debug = False):
		"""
        Init method
        Input: optional debug parameter which toogles debug messages
        Output: none
        """
		self.debug = debug
		self.data_downloaded = False
		self.result = None

	def download_zones(self):
		"""
		Downloads the no-fly zones from droneluftrum.dk
		Input: none
		Output: none but stores the data as class variable
		"""
		get_token_url = "https://www.droneluftrum.dk/oauth/token?grant_type=client_credentials"
		required_auth = HTTPBasicAuth('NaviairDroneWeb','NaviairDroneWeb')
		payload = {}
		# r = requests.post(url = get_token_url, verify=True, auth=required_auth, data=payload)
		try:
			r = requests.post(url = get_token_url,auth=required_auth,data=payload)
		except:
			if self.debug:
				print 'Unexpected error in polling get_requests'

		if r.text == '0' or int(r.status_code) != 200: # Handle empty response
			if self.debug:
				print "No response or error code 200"
		else:
			if self.print_raw_response == True:
				print "--- Raw response start ---"
				print r.text
				print "--- Raw response end ---"
			try:
				jsonformat = json.loads(r.text) # convert to json
			except:
				if self.debug:
					print 'Unexpected error in parsing r.text as json'

			access_token = jsonformat["access_token"]
			token_type   = jsonformat["token_type"]

			if access_token != '' and access_token != '0':
				# print "Access token: "+jsonformat["access_token"]
				if self.debug:
					print "Access token acquired"

				#time.sleep(1)

				# # Get no-fly zones
				# Parse URL - NOTE added an extra '%' to escape '%20', could have been replaced with the corresponding ' ' (space)
				url = 'https://www.droneluftrum.dk//api/uaszones/exportKmlUasZones?Authorization=%s%%20%s' % (token_type, access_token)
				#print 'Trying URL: %s' % url
				if self.debug:
					print 'Attempting to download'

				try:
					response = urlopen(url)
				except HTTPError as e:
					self.result = '%s' % (e.code)
				except URLError as e:
					self.result = '%s %s' % (e.code, e.reason)
				else:
					if self.debug:
						print 'No errors encountered during download, attempting to read result'
					self.result = response.read()

				if self.result == '400' or self.result == 400:
					if self.debug:
						print 'Bad request, terminating'
					sys.exit()
				else:
					self.data_downloaded = True
					if self.debug:
						print 'Result read'

	def save_to_file(self):
		"""
		Saves the downloaded data to a file
		Input: none
		Output: none but saves a file containing the no-fly zones in KML format
		"""
		if self.data_downloaded:
			# Get datetime information (https://www.saltycrane.com/blog/2008/06/how-to-get-current-date-and-time-in/)
			now = datetime.datetime.now()
			# Save to file with proper naming
			if self.debug:
				print 'Attempting to write file'
			file_name = 'KmlUasZones_%d-%02d-%02d-%02d-%02d.kml' % (now.year, now.month, now.day, now.hour, now.minute)
			file = open(file_name, 'w')
			file.write(self.result)
			file.close()
			if self.debug:
				print 'File written: %s' % file_name
			return file_name
		else:
			return None

	def get_data(self):
		"""
		Returns the downloaded data
		Input: none
		Output: the downloaded data
		"""
		if self.data_downloaded:
			return self.result
		else:
			return False

if __name__ == '__main__':
	test = get_no_fly_zones(True)
	test.download_zones()
	print test.save_to_file()
	test.get_data()
