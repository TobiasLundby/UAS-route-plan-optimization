#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: various helper functions and defines
    License: BSD 3-Clause
"""

""" Terminal output color constants """
default_term_color_info         = 'cyan'
default_term_color_info_alt     = 'magenta'
info_alt_indent                 = ' -- '
default_term_color_error        = 'red'
error_indent                    = ' ÷÷ '
default_term_color_tmp_res      = 'yellow'
tmp_res_indent                  = ' -> '
default_term_color_res          = 'green'
res_indent                      = ' ++ '

""" Other constants """
FOREVER                         = 60*60*24*365*100  # unit: s; 100 years excl. leap year
INF                             = 4294967295 # 32bit from 0
