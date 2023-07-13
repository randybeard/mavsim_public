"""
point_gimbal
    - point gimbal at target
part of mavsim
    - Beard & McLain, PUP, 2012
    - Update history:  
        3/31/2022 - RWB
"""
import numpy as np
from tools.rotations import Euler2Rotation
import parameters.camera_parameters as CAM


class Gimbal:
    def pointAtGround(self, mav):
        ###### TODO #######
        # desired inertial frame vector points down
        
        # rotate line-of-sight vector into body frame and normalize
        
        ell = np.array([[0],[0],[0]])
        return( self.pointAlongVector(ell, mav.camera_az, mav.camera_el) )

    def pointAtPosition(self, mav, target_position):
        ###### TODO #######
        # line-of-sight vector in the inertial frame
        
        # rotate line-of-sight vector into body frame and normalize
        ell = np.array([[0],[0],[0]])
        return( self.pointAlongVector(ell, mav.camera_az, mav.camera_el) )

    def pointAlongVector(self, ell, azimuth, elevation):
        # point gimbal so that optical axis aligns with unit vector ell
        # ell is assumed to be aligned in the body frame
        # given current azimuth and elevation angles of the gimbal

        ##### TODO #####
        # compute control inputs to align gimbal
        
        # proportional control for gimbal
        u_az = 0
        u_el = 0
        return( np.array([[u_az], [u_el]]) )




