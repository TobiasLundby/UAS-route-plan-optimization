#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-01-29 TL First version
2018-01-30 TL Refined version with safety measures for internet connection and empty response
2018-01-12 TL Made into class for use in other programs
"""

"""
Description:
Tools for working with internet
License: BSD 3-Clause
"""

#print 'Importing libraries'
from urllib2 import urlopen, URLError, HTTPError
#print 'Import done\n'

default_test_ip = 'http://216.58.207.206'

class internet_tools():
    def __init__(self, in_test_addr = default_test_ip):
        if isinstance(in_test_addr, str):
            self.test_addr = in_test_addr
        else:
            self.test_addr = default_test_ip
    def internet_on(self, timeout_in = 2):
        # Based on code snippet from unutbu (https://stackoverflow.com/users/190597/unutbu)
        # IP find by: 'dig google.com +trace'
        try:
            urlopen(self.test_addr, timeout=timeout_in)
            return True
        except URLError as err:
            return False

if __name__ == '__main__':
    # Run self test
    test = internet_tools()
    print test.internet_on()
