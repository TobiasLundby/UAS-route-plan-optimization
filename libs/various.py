#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: various helper functions and defines
    License: BSD 3-Clause
"""

from time import time
from datetime import datetime, timedelta
from pytz import UTC # timezones in the datetime format

""" Terminal output color constants """
TERM_COLOR_INFO                 = 'cyan'
TERM_COLOR_INFO_ALT             = 'magenta'
INFO_ALT_INDENT                 = ' -- '
TERM_COLOR_ERROR                = 'red'
ERROR_INDENT                    = ' ÷÷ '
TERM_COLOR_SUB_RES              = 'yellow'
SUB_RES_INDENT                  = ' -> '
TERM_COLOR_RES                  = 'green'
RES_INDENT                      = ' ++ '

""" Other constants """
FOREVER                         = 60*60*24*365*100  # unit: s; 100 years excl. leap year
INF                             = 4294967295 # 32bit from 0

""" Helper functions """
def get_cur_time_epoch():
    """ Returns the current EPOCH time """
    return time()
def get_cur_time_epoch_wo_us():
    """ Returns the current EPOCH time stripped of micro seconds """
    return round(get_cur_time_epoch(),3)
def get_cur_time_human_UTC():
    """ Returns the current UTC time in human readable time format """
    return datetime.fromtimestamp( get_cur_time_epoch(), UTC ).strftime('%Y-%m-%d %H:%M:%S')
def get_cur_time_human_local():
    """ Returns the current local time in human readable time format """
    return datetime.fromtimestamp( get_cur_time_epoch() ).strftime('%Y-%m-%d %H:%M:%S')
def convert_epoch_time2human_UTC(epoch_time = 0):
    """ Converts EPOCH time into UTC human redable time format """
    return datetime.fromtimestamp( epoch_time ).strftime('%Y-%m-%d %H:%M:%S')
def convert_epoch_time2human_UTC(epoch_time = 0):
    """ Converts EPOCH time into local human redable time format """
    return datetime.fromtimestamp( epoch_time, UTC ).strftime('%Y-%m-%d %H:%M:%S')
def convert_time_delta2human(time_delta):
    return timedelta(seconds=time_delta)
def get_cur_time_datetime():
    return datetime.now()
