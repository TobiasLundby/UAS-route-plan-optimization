#!/usr/bin/env python
#/****************************************************************************
# Elevation lookup based on SRTM data
# Copyright (c) 2017, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# All rights reserved.
#
# Released under the BSD 3-Clause licence:
# Redistribution and use in source and binary forms, with or withoutn
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# Part of this work (test routine) has been derived from the srtm-python
# software "A python program that reads SRTM Digital Elevation Model files"
# Copyright (c) 2015 Aatish Neupane
# Released under the MIT license
# https://github.com/aatishnn/srtm-python
#
# Part of this work (srtm_files.json) has been derived from the srtm.py
# software "Geo elevation data parser for "The Shuttle Radar Topography Mission"
# data"
# Copyright (c) Tomo Krajina
# Released under the Apache 2.0 license
# https://github.com/tkrajina/srtm.py
#
#****************************************************************************/
"""
Based on data from the Shuttle Radar Tophography Mission
https://www2.jpl.nasa.gov/srtm/

The names of individual data tiles refer to the longitude and latitude of the
lower-left (southwest) corner of the tile (this follows the DTED convention
as opposed to the GTOPO30 standard). For example, the coordinates of the
lower-left corner of tile N40W118 are 40 degrees north latitude and 118 degrees
west longitude.

The data are stored in row major order (all the data for row  1, followed by
all the data for row 2

Heights are in meters referenced to the WGS84/EGM96 geoid. Data voids are
assigned the value -32768
https://dds.cr.usgs.gov/srtm/version2_1/Documentation/Quickstart.pdf
https://dds.cr.usgs.gov/srtm/version2_1/Documentation/SRTM_Topo.pdf

2018-01-08 KJ First version
2018-05-03 TL Changed to use urllib2 instead of urllib to better handle missing
			internet connections but requires a couple of lines more
2018-05-04 TL Changed output messages so they are only displayed if the debug
			parameter is set to True
"""
import struct
from math import sqrt

# the below imports are used by self.download_tile
import json
import urllib #(Python 3+ use 'import urllib.request' and urllib.request.urlretrieve)
from urllib2 import urlopen, URLError, HTTPError
import zipfile
import os

class srtm_lib():
	def __init__(self, dir_tiles = '', debug = False):
		self.dir_tiles = dir_tiles
		self.tile_data = []
		self.tile_name = []
		self.tile_sqlen = []
		self.srtm_urls = []

		self.debug = debug

	def get_tile_name(self, lat, lon):
		if lat >= 0:
			ns = 'N'
			lat_f = int(abs(lat))
		else:
			ns= 'S'
			lat_f = int(abs(lat)) + 1

		if lon >= 0:
			ew = 'E'
			lon_f = int(abs(lon))
		else:
			ew = 'W'
			lon_f = int(abs(lon)) + 1

		name = '%s%02d%s%03d' % (ns, lat_f, ew, lon_f)
		return name

	def download_tile(self, tile_name):
		if self.srtm_urls == []:
			self.srtm_urls = json.load(open(self.dir_tiles + 'srtm_urls.json'))
		file_name = tile_name + '.hgt'
		path_zipped = self.dir_tiles + tile_name + '.zip'
		url = self.srtm_urls['srtm3'][file_name]
		if self.debug:
			print 'Downloading:', url
		#urllib.urlretrieve (url, path_zipped)
		try:
			f = urlopen(url)
		except HTTPError, e:
			print 'The server couldn\'t fulfill the request.'
			print 'Error code: ', e.code
			exit(1)
		except URLError, e:
			print 'Failed to reach server.'
			print 'Reason: ', e.reason
			exit(1)
		except IOError, e:
			if hasattr(e, 'reason'):
				print 'Failed to reach server.'
				print 'Reason: ', e.reason
			elif hasattr(e, 'code'):
				print 'The server couldn\'t fulfill the request.'
				print 'Error code: ', e.code
			exit(1)
		else:
			data = f.read()
			with open(path_zipped, "wb") as code:
				code.write(data)
			if self.debug:
				print 'Unzipping'
			zip_ref = zipfile.ZipFile(path_zipped, 'r')
			zip_ref.extractall(self.dir_tiles)
			zip_ref.close()
			if self.debug:
				print 'Deleting zip archive'
			os.remove (path_zipped)
			#return self.srtm_urls['srtm3'][file_name]

	def load_tile_data(self, tile_name):
		result = False
		data = []
		path_tile = self.dir_tiles + tile_name + '.hgt'
		try:
			f = open(path_tile, "rb")
		except IOError:
			if self.debug:
				print 'File does not appear to exist'
			self.download_tile(tile_name)
			f = open(path_tile, "rb")
		try:
			h = f.read(2)
			while h != "":
				data.append(struct.unpack('>h', h)[0])
				h = f.read(2)
		finally:
			f.close()
			result = True
			self.tile_name.append(tile_name)
			self.tile_data.append(data)
			self.tile_sqlen.append(int(sqrt(len(data))))
		return result

	def get_tile_cache_index (self, lat, lon):
		index = -1
		tile_name = self.get_tile_name (lat, lon) # determine tile name
		for i in range(len(self.tile_name)):
			if tile_name == self.tile_name[i]:
				index = i
		if index == -1: # tile not cached yet
			if self.load_tile_data (tile_name): # try to load file
				index = len(self.tile_data) - 1
		return index

	def get_nearest_elevation(self, lat, lon, index):
		sqlen = self.tile_sqlen[index]
		row = int(round((lat - int(lat)) * (sqlen - 1)))
		if row < 0:
			row = sqlen + row # southern latitudes
		row = sqlen - 1 - row
		col = int(round((lon - int(lon)) * (sqlen - 1)))
		if col < 0:
			col = sqlen - 1 + col # western longitudes
		pos = row * (sqlen) + col
		#print 'row, col', row, col
		return self.tile_data[index][pos]

	def get_elevation(self, lat, lon):
		elevation = -32768
		index = self.get_tile_cache_index (lat, lon)
		if index != -1:
			elevation = self.get_nearest_elevation (lat, lon, index)
 		return elevation

if __name__ == "__main__":
	dir_tiles = '' # remember trailing backslash
	srtm = srtm_lib(dir_tiles)
	print 'Ngelahun, SL (260m):', srtm.get_elevation (8.00182456, -11.0618724)
	print 'Very close to Odense harbor:', srtm.get_elevation (55.431122, 10.420436)
	print 'Close to Odense harbor:', srtm.get_elevation (55.427203, 10.419043)
