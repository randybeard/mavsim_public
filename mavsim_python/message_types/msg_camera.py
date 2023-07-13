"""
msg_camera
    - messages type output of camera
    
part of mavsim
    - Beard & McLain, PUP, 2012
    - Updated:
        4/1/2022 - RWB
"""
import parameters.camera_parameters as CAM


class MsgCamera:
    def __init__(self):
        self.pixel_x = 0
        self.pixel_y = 0
        self.size_x = CAM.pix
        self.size_y = CAM.pix
        self.fov = CAM.fov
        self.focal_length = CAM.f
