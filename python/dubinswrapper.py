#!/usr/bin/env python3

import os, sys, platform, math
import ctypes as ct
import numpy as np

class DubinsWrapper:

    libgdip = None

    def init_library(self):
        try:
            file_extension = '.so'
            if platform.system() =='cli':
                file_extension = '.dll'
            elif platform.system() =='Windows':
                file_extension = '.dll'
            elif platform.system() == 'Darwin':
                file_extension = '.dylib'
            else:
                file_extension = '.so'

            libfullpath = os.path.abspath(os.path.join(os.path.dirname(__file__), '../gdip/lib/libGDIP' + file_extension))
            print("Loading " + libfullpath)
            DubinsWrapper.libgdip = ct.CDLL(libfullpath)
        except:
            print ('----------------------------------------------------')
            print ('The GDIP library could not be loaded.')
            print ('----------------------------------------------------')
            exit(1)
        
        DubinsWrapper.libgdip.new_GdipAPI.restype = ct.c_void_p
    
    def __del__(self):
        #print("Destruct Dubins maneuver")
        self.gdip_destruct(self.object_hanle)
        
    def __init__(self):

        #initialize the GDIP library (only for the first time)
        if DubinsWrapper.libgdip is None:
            self.init_library()
        
        # create instance of the maneuver
        self.object_hanle = self.libgdip.new_GdipAPI()

        self.gdip_set_configurations = ct.CFUNCTYPE \
            (None, ct.c_void_p, ct.c_double*3, ct.c_double*3, ct.c_double) \
            (("python_set_configurations", DubinsWrapper.libgdip))

        self.gdip_set_configurations_dip = ct.CFUNCTYPE \
            (None, ct.c_void_p, ct.c_double*2, ct.c_double*2, ct.c_double*2, ct.c_double*2, ct.c_double) \
            (("python_set_configurations_dip", DubinsWrapper.libgdip))
        
        self.gdip_set_configurations_gdip = ct.CFUNCTYPE \
            (None, ct.c_void_p, ct.c_double*2, ct.c_double*2, ct.c_double, ct.c_double*2, ct.c_double*2, ct.c_double, ct.c_double) \
            (("python_set_configurations_gdip", DubinsWrapper.libgdip))

        self.gdip_get_length = ct.CFUNCTYPE (ct.c_double, ct.c_void_p) (("python_get_length", DubinsWrapper.libgdip))

        self.gdip_sample_state_to_tmp = ct.CFUNCTYPE (None, ct.c_void_p, ct.c_double) (("python_sample_state_to_tmp", DubinsWrapper.libgdip))

        self.gdip_get_tmp_x = ct.CFUNCTYPE(ct.c_double, ct.c_void_p) (("python_get_tmp_x", DubinsWrapper.libgdip))
        self.gdip_get_tmp_y = ct.CFUNCTYPE(ct.c_double, ct.c_void_p) (("python_get_tmp_y", DubinsWrapper.libgdip))
        self.gdip_get_tmp_theta = ct.CFUNCTYPE(ct.c_double, ct.c_void_p) (("python_get_tmp_theta", DubinsWrapper.libgdip))

        self.gdip_destruct = ct.CFUNCTYPE(None, ct.c_void_p) (("python_destruct", DubinsWrapper.libgdip))

    @staticmethod
    def shortest_path(start, end, turning_radius):
        """
        Construct Dubins maneuver between two configurations

        Parameters
        ----------
        start: numpy array(double*3)
            start configuration
        end: numpy array(double*3)
            end configuration
        turning_radius: double
            minimum turning radius
        """
        man = DubinsWrapper()

        arrtype = ct.c_double * 3
        arr_p1 = arrtype()
        arr_p2 = arrtype()
        c_radius = ct.c_double()
        for i in range(0,3):
            arr_p1[i] = start[i]
            arr_p2[i] = end[i]
        c_radius = turning_radius
        
        man.gdip_set_configurations(man.object_hanle, arr_p1, arr_p2, c_radius)
        return man

    @staticmethod
    def shortest_path_DIP(point1, interval1, point2, interval2, turning_radius):
        """
        Construct Dubins maneuver between two configurations

        Parameters
        ----------
        point1: numpy array(double*2)
            start position
        interval1: numpy array(double*2)
            angle interval for point1 (right_angle, diff)
            the interval is then [right_angle, right_angle + diff]
        point2: numpy array(double*2)
            end position
        interval2: numpy array(double*2)
            angle interval for point2 (right_angle, diff)
            the interval is then [right_angle, right_angle + diff]
        turning_radius: double
            minimum turning radius
        """
        man = DubinsWrapper()

        arrtype = ct.c_double * 2
        arr_p1 = arrtype()
        arr_i1 = arrtype()
        arr_p2 = arrtype()
        arr_i2 = arrtype()
        c_radius = ct.c_double()

        arr_p1[0] = point1[0]
        arr_p1[1] = point1[1]

        arr_i1[0] = interval1[0]
        arr_i1[1] = interval1[1]

        arr_p2[0] = point2[0]
        arr_p2[1] = point2[1]

        arr_i2[0] = interval2[0]
        arr_i2[1] = interval2[1]

        c_radius = turning_radius
        
        man.gdip_set_configurations_dip(man.object_hanle, arr_p1, arr_i1, arr_p2, arr_i2, c_radius)
        return man

    @staticmethod
    def shortest_path_GDIP(point1, interval1, radius1, point2, interval2, radius2, turning_radius):
        """
        Construct Dubins maneuver between two configurations

        Parameters
        ----------
        point1: numpy array(double*2)
            start position
        interval1: numpy array(double*2)
            angle interval for point1 (right_angle, diff)
            the interval is then [right_angle, right_angle + diff]
        point2: numpy array(double*2)
            end position
        interval2: numpy array(double*2)
            angle interval for point2 (right_angle, diff)
            the interval is then [right_angle, right_angle + diff]
        turning_radius: double
            minimum turning radius
        """
        man = DubinsWrapper()

        arrtype = ct.c_double * 2
        arr_p1 = arrtype()
        arr_i1 = arrtype()
        arr_p2 = arrtype()
        arr_i2 = arrtype()

        arr_p1[0] = point1[0]
        arr_p1[1] = point1[1]

        arr_i1[0] = interval1[0]
        arr_i1[1] = interval1[1]

        arr_p2[0] = point2[0]
        arr_p2[1] = point2[1]

        arr_i2[0] = interval2[0]
        arr_i2[1] = interval2[1]

        c_radius = ct.c_double()
        c_radius = turning_radius

        c_radius1 = ct.c_double()
        c_radius1 = radius1

        c_radius2 = ct.c_double()
        c_radius2 = radius2
        
        man.gdip_set_configurations_gdip(man.object_hanle, arr_p1, arr_i1, c_radius1, arr_p2, arr_i2, c_radius2, c_radius)
        return man

    def get_length(self):
        """
        Get length of the maneuver

        Returns
        -------
        bool
            True if there is collision, False otherwise
        """
        return self.gdip_get_length(self.object_hanle)

    def sample_many(self, step):
        """
        Sample the manuver based on the step

        Parameters
        ----------
        step: double
            step for the sampling

        Returns
        -------
        states
        """

        length = self.get_length()
        lens = np.arange(0, length, step)   
        path = []

        for l in lens:
            self.gdip_sample_state_to_tmp(self.object_hanle, l)
            state = [self.gdip_get_tmp_x(self.object_hanle), self.gdip_get_tmp_y(self.object_hanle), self.gdip_get_tmp_theta(self.object_hanle)]
            path.append(state)

        return [path, lens]

if __name__ == "__main__":
    a = DubinsWrapper()
    a.shortest_path((0,0,0), (1,1,2), 1)
    print(a.get_length())