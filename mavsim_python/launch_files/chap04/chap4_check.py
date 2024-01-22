"""
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta
from models.wind_simulation import WindSimulation

wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)

delta = MsgDelta()
delta.elevator = -0.2
delta.aileron = 0.0
delta.rudder = 0.005
delta.throttle = 0.5

T_p, Q_p = mav._motor_thrust_torque(mav._Va, delta.throttle)
print("Propeller Forces and Torque", "\n")
print("T_p: " , T_p)
print("Q_p: " , Q_p, "\n\n")

forcesAndMoments = mav._forces_moments(delta)
print("Forces and Moments : Case 1", "\n")
print("fx: " , forcesAndMoments.item(0))
print("fy: " , forcesAndMoments.item(1))
print("fz: " , forcesAndMoments.item(2))
print("Mx: " , forcesAndMoments.item(3))
print("My: " , forcesAndMoments.item(4))
print("Mz: " , forcesAndMoments.item(5) , "\n\n")

x_dot = mav._f(mav._state, forcesAndMoments)
print("State Derivatives : Case 1", "\n")
print("north_dot: ", x_dot.item(0))
print("east_dot: ", x_dot.item(1))
print("down_dot: ", x_dot.item(2))
print("   u_dot: ", x_dot.item(3))
print("   v_dot: ", x_dot.item(4))
print("   w_dot: ", x_dot.item(5))
print("  e0_dot: ", x_dot.item(6))
print("  e1_dot: ", x_dot.item(7))
print("  e2_dot: ", x_dot.item(8))
print("  e3_dot: ", x_dot.item(9))
print("   p_dot: ", x_dot.item(10))
print("   q_dot: ", x_dot.item(11))
print("    r_dt: ", x_dot.item(12) , "\n\n\n")

##### Case 1 ######

delta.elevator = -0.15705144
delta.aileron = 0.01788999
delta.rudder = 0.01084654
delta.throttle = 1.

mav._state = np.array([[ 6.19506532e+01],
 [ 2.22940203e+01],
 [-1.10837551e+02],
 [ 2.73465947e+01],
 [ 6.19628233e-01],
 [ 1.42257772e+00],
 [ 9.38688796e-01],
 [ 2.47421558e-01],
 [ 6.56821468e-02],
 [ 2.30936730e-01],
 [ 4.98772167e-03],
 [ 1.68736005e-01],
 [ 1.71797313e-01]])

T_p, Q_p = mav._motor_thrust_torque(mav._Va, delta.throttle)
print("Propeller Forces and Torque", "\n")
print("T_p: " , T_p)
print("Q_p: " , Q_p, "\n\n")

forcesAndMoments = mav._forces_moments(delta)
print("Forces and Moments : Case 2" , "\n")
print("fx: " , forcesAndMoments.item(0))
print("fy: " , forcesAndMoments.item(1))
print("fz: " , forcesAndMoments.item(2))
print("Mx: " , forcesAndMoments.item(3))
print("My: " , forcesAndMoments.item(4))
print("Mz: " , forcesAndMoments.item(5) , "\n\n")

x_dot = mav._f(mav._state, forcesAndMoments)
print("State Derivatives : Case 2", "\n")
print("north_dot: ", x_dot.item(0))
print("east_dot: ", x_dot.item(1))
print("down_dot: ", x_dot.item(2))
print("   u_dot: ", x_dot.item(3))
print("   v_dot: ", x_dot.item(4))
print("   w_dot: ", x_dot.item(5))
print("  e0_dot: ", x_dot.item(6))
print("  e1_dot: ", x_dot.item(7))
print("  e2_dot: ", x_dot.item(8))
print("  e3_dot: ", x_dot.item(9))
print("   p_dot: ", x_dot.item(10))
print("   q_dot: ", x_dot.item(11))
print("    r_dt: ", x_dot.item(12) , "\n\n\n")


current_wind = np.array([[ 0.        ],
 [ 0.        ],
 [ 0.        ],
 [-0.00165177],
 [-0.00475441],
 [-0.01717199]])


mav._update_velocity_data(current_wind)
print("Wind Update" , "\n")
print("Va: ", mav._Va)
print("alpha: ", mav._alpha)
print("beta: ", mav._beta , "\n\n")

T_p, Q_p = mav._motor_thrust_torque(mav._Va, delta.throttle)
print("Propeller Forces and Torque", "\n")
print("T_p: " , T_p)
print("Q_p: " , Q_p, "\n\n")

forcesAndMoments = mav._forces_moments(delta)
print("Forces and Moments : Case w/Wind" , "\n")
print("fx: " , forcesAndMoments.item(0))
print("fy: " , forcesAndMoments.item(1))
print("fz: " , forcesAndMoments.item(2))
print("Mx: " , forcesAndMoments.item(3))
print("My: " , forcesAndMoments.item(4))
print("Mz: " , forcesAndMoments.item(5) , "\n\n")

x_dot = mav._f(mav._state, forcesAndMoments)
print("State Derivatives : Case w/Wind", "\n")
print("north_dot: ", x_dot.item(0))
print("east_dot: ", x_dot.item(1))
print("down_dot: ", x_dot.item(2))
print("   u_dot: ", x_dot.item(3))
print("   v_dot: ", x_dot.item(4))
print("   w_dot: ", x_dot.item(5))
print("  e0_dot: ", x_dot.item(6))
print("  e1_dot: ", x_dot.item(7))
print("  e2_dot: ", x_dot.item(8))
print("  e3_dot: ", x_dot.item(9))
print("   p_dot: ", x_dot.item(10))
print("   q_dot: ", x_dot.item(11))
print("    r_dt: ", x_dot.item(12) , "\n\n\n")


# These outputs are results using the default parameters from the imported parameter files

# Propeller Forces and Torque 

# T_p:  -12.43072534597213
# Q_p:  -0.49879620097737787 


# Forces and Moments : Case 1 

# fx:  -12.109717001006562
# fy:  0.20707328125000002
# fz:  63.44373750624077
# Mx:  0.5063701133123779
# My:  8.75643373378125
# Mz:  -0.21774997963125006 


# State Derivatives : Case 1 

# north_dot:  25.0
# east_dot:  0.0
# down_dot:  0.0
#    u_dot:  -1.1008833637278692
#    v_dot:  0.01882484375
#    w_dot:  5.767612500567343
#   e0_dot:  -0.0
#   e1_dot:  0.0
#   e2_dot:  0.0
#   e3_dot:  0.0
#    p_dot:  0.6021690003674433
#    q_dot:  7.714919589234582
#     r_dt:  -0.08257466286924951 



# Propeller Forces and Torque 

# T_p:  37.7794805541605
# Q_p:  1.8098467397878482 


# Forces and Moments : Case 2 

# fx:  36.99938467421735
# fy:  54.13991070468528
# fz:  46.97294443218653
# Mx:  1.6030203500067555
# My:  5.982053219886495
# Mz:  -1.1805441645292776 


# State Derivatives : Case 2 

# north_dot:  24.283238643486627
# east_dot:  12.605130052025968
# down_dot:  1.2957327060769266
#    u_dot:  3.2299908091423797
#    v_dot:  0.2308339966235602
#    w_dot:  8.881532282520418
#   e0_dot:  -0.025995661302161892
#   e1_dot:  -0.011500703223228347
#   e2_dot:  0.05851804333262313
#   e3_dot:  0.10134276693843723
#    p_dot:  1.8427420637214973
#    q_dot:  5.2743652738342774
#     r_dt:  -0.5471458931221012 



# Wind Update 

# Va:  27.39323489287441
# alpha:  0.05259649205640062
# beta:  0.022801214339060967 


# Propeller Forces and Torque 

# T_p:  31.31315544701058
# Q_p:  1.58778287798956 


# Forces and Moments : Case w/Wind 

# fx:  36.22803068339798
# fy:  48.44092504137796
# fz:  -39.39246596662818
# Mx:  0.10867448074086083
# My:  0.1249623335264915
# Mz:  -0.09481002421995177 


# State Derivatives : Case w/Wind 

# north_dot:  24.283238643486627
# east_dot:  12.605130052025968
# down_dot:  1.2957327060769266
#    u_dot:  3.1598677190678917
#    v_dot:  -0.28725560913165094
#    w_dot:  1.0301313371736245
#   e0_dot:  -0.025995661302161892
#   e1_dot:  -0.011500703223228347
#   e2_dot:  0.05851804333262313
#   e3_dot:  0.10134276693843723
#    p_dot:  0.10284849278240359
#    q_dot:  0.11393277483867911
#     r_dt:  -0.04899299126408019 