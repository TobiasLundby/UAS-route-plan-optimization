#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description: memory usage class
    License: BSD 3-Clause
"""

""" Import libraries """
from guppy import hpy # for getting heap info

class memory_usage():
    def __init__(self):
        """
        Init method
        Input: none
        Output: none
        """
        self.heap_saved = False
        self.heap_start_saved = False

    def __save_heap(self):
        """
        Saves the heap to internal class variable
        Input: none
        Output: none
        """
        h = hpy()
        heap = h.heap() # Extract heap
        heap_str = str(heap) # Convert to string to make a deepcopy so it is not just pointing to an object which changes
        heap_saved = True
        return heap_str, heap_saved

    def save_heap(self):
        self.heap_str, self.heap_saved = self.__save_heap()

    def save_heap_start(self):
        self.heap_start_str, self.heap_start_saved = self.__save_heap()

    def get_bytes(self):
        """
        Returns the bytes used
        Input: none
        Output: bytes used [int] (returns -1 if heap has not been saved)
        """
        if self.heap_saved:
            index_start_bytes = self.heap_str.find('Total size = ')
            index_end_bytes = self.heap_str.find(' bytes.')
            return int(self.heap_str[index_start_bytes+13:index_end_bytes])
        else:
            print 'Please save the heap before calling'
            return -1

    def get_objects(self):
        """
        Returns the objects used
        Input: none
        Output: objects used [int] (returns -1 if heap has not been saved)
        """
        if self.heap_saved:
            index_end_objects = self.heap_str.find(' objects')
            return int(self.heap_str[22:index_end_objects])
        else:
            print 'Please save the heap before calling'
            return -1

    def get_start_bytes(self):
        """
        Returns the bytes used from start
        Input: none
        Output: bytes used [int] (returns -1 if heap has not been saved)
        """
        if self.heap_start_saved:
            index_start_bytes = self.heap_start_str.find('Total size = ')
            index_end_bytes = self.heap_start_str.find(' bytes.')
            return int(self.heap_start_str[index_start_bytes+13:index_end_bytes])
        else:
            print 'Please save the heap before calling'
            return -1

    def get_start_objects(self):
        """
        Returns the objects used from start
        Input: none
        Output: objects used [int] (returns -1 if heap has not been saved)
        """
        if self.heap_start_saved:
            index_end_objects = self.heap_start_str.find(' objects')
            return int(self.heap_start_str[22:index_end_objects])
        else:
            print 'Please save the heap before calling'
            return -1

    def get_bytes_diff(self):
        """
        Returns the byte difference between the finish and the start
        Input: none
        Output: bytes used [int] (returns -1 if heap has not been saved)
        """
        bytes_start = self.get_start_bytes()
        bytes_finish = self.get_bytes()
        if bytes_start != -1 and bytes_finish != -1:
            return bytes_finish-bytes_start
        else:
            print 'Please save the start and finish heap before calling'
            return -1

    def get_objects_diff(self):
        """
        Returns the object difference between the last heap saving and the start heap saving
        Input: none
        Output: objects used [int] (returns -1 if heap has not been saved)
        """
        objects_start = self.get_start_objects()
        objects_finish = self.get_objects()
        if objects_start != -1 and objects_finish != -1:
            return objects_finish-objects_start
        else:
            print 'Please save the start and finish heap before calling'
            return -1
