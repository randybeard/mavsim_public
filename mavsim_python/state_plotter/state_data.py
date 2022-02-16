#!/usr/bin/env python
import numpy as np

class StateData():
    def __init__(self, sigma_bounds=None, max_length=None, is_angle=False, rad2deg=False):
        self.data = []
        self.time = []
        self.max_length = max_length
        self.is_angle = is_angle
        self.rad2deg = rad2deg
        self.sigma_bounds = sigma_bounds
        self.sigma_data = {}
        self.current_sigma = 0.0 # Helps with 2D plot sigma bounds
        self.has_sigma = (self.sigma_bounds is not None)
        if self.has_sigma:
            for bound in self.sigma_bounds:
                self.sigma_data[bound] = {'lower':[], 'upper':[]}

    def add_data(self, data, t, sigma=0):
        if self.is_angle:
            data = angle_wrap(data)
        if self.rad2deg:
            data, sigma = np.degrees([data, sigma])
        self.data.append(data)
        self.time.append(t)
        if self.has_sigma:
            for bound in self.sigma_bounds:
                self.sigma_data[bound]['lower'].append(data - bound*sigma)
                self.sigma_data[bound]['upper'].append(data + bound*sigma)
            self.current_sigma = sigma
        if self.max_length is not None and len(self.data) > self.max_length:
            self.pop(0)

    def set_data(self, data, t, sigma=None):
        # Make sure the lists are the same size
        if len(data) != len(t):
            raise ValueError('Length of data ({}) does not match length of t ({}).'.format(len(data), len(t)))
        elif sigma is not None and len(sigma) != len(t):
            raise ValueError('Length of sigma ({}) does not match length of t ({}).'.format(len(sigma), len(t)))
        # Populate empty sigma if not given
        if sigma is None:
            sigma = np.zeros_like(data)
        if self.is_angle:
            data = angle_wrap(data)
        if self.rad2deg:
            data = np.degrees(data)
            sigma = np.degrees(sigma)
        self.data = data
        self.time = t
        if self.has_sigma:
            for bound in self.sigma_bounds:
                for bound in self.sigma_bounds:
                    self.sigma_data[bound]['lower'] = list(data - bound*sigma)
                    self.sigma_data[bound]['upper'] = list(data + bound*sigma)
                self.current_sigma = sigma[-1]

    def get_data_vec(self):
        return self.data

    def get_time_vec(self):
        return self.time

    def get_sigma_data(self):
        return self.sigma_data

    def get_current_sigma(self):
        return self.current_sigma

    def pop(self, idx=-1):
        self.data.pop(idx)
        self.time.pop(idx)
        for data in self.sigma_data.values():
            data.pop(idx)

def angle_wrap(x):
    xwrap = np.array(np.mod(x, 2*np.pi))
    mask = np.abs(xwrap) > np.pi
    xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
    if np.size(xwrap) == 1:
        return float(xwrap)
    else:
        return xwrap
