import numpy as np

# gimbal parameters
az0 = np.radians(0)      # initial azimuth angle
el0 = np.radians(-90)  # initial elevation angle (pointing down)
az_limit = np.radians(180)  # azimuth angle limit
el_limit = np.radians(180)  # elevation angle limit
az_gain = 1  # gain on azimuth dynamics (azdot = az_gain*u_az)
el_gain = 1  # gain on elevation dynamics (eldot = el_gain*u_el)
k_az = 100  # proportional control gain for gimbal azimuth
k_el = 100  # proportional control gain for gimbal elevation

# camera parameters
fps = 10  # frames per second
pix = 480  # size of (square) pixel array
fov = np.radians(10)  # field of view of camera
f = 480 # (pix / 2) / np.tan(fov / 2)  # focal range
sigma_pixel = 2  # (pixels) - standard of the pixel noise