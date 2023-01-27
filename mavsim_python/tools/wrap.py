"""
wrap chi_1, so that it is within +-pi of chi_2
"""
import numpy as np

def wrap(chi_1, chi_2):
    while chi_1 - chi_2 > np.pi:
        chi_1 = chi_1 - 2.0 * np.pi
    while chi_1 - chi_2 < -np.pi:
        chi_1 = chi_1 + 2.0 * np.pi
    return chi_1
