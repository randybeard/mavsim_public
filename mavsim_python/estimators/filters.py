# """
# filters
#     - Beard & McLain, PUP, 2012
#     - Last Update:
#         2/19/2019 - RWB
#         3/6/2024 - RWB
# """
import numpy as np
from typing import Callable


class AlphaFilter:
    '''
    alpha filter implements a simple digital low pass filter
        y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    '''
    def __init__(self, alpha: float=0.5, y0: float=0.0):
        '''
        Parameters
            alpha: float
                alpha filter gain 0 <= alpha <= 1
            y0: float
                initial output of the filter
        '''
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u: float)->float:
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y


class ExtendedKalmanFilterContinuousDiscrete:
    ''' 
        Continous-discrete extended Kalman filter (EKF)
        Assumes continuous dyamamics
            xdot = f(x, u) + q(t)
        and discrete measurments
            y[k] = h(x[k]) + r[k]
        where q and r are zero mean Gaussian with covariances Q and R
    '''
    def __init__(self, 
                 f: Callable,  
                 Q: np.ndarray, 
                 P0: np.ndarray, 
                 xhat0: np.ndarray, 
                 Qu: np.ndarray, 
                 Ts: float,
                 N: float=10):
        '''
            Initialize Ekf class

            Parameters:
                f : Callable function
                    process model xdot=f(x,u)
                    x in R^n, u in R^p
                Q : numpy ndarray (nxn)
                    covariance of the process noise
                P0 : numpy ndarray (nxn)
                    initial covariance of the estimation error
                xhat0 : numpy ndarray (nxn)
                    initial estimate of the state
                Qu: numpy ndarray (pxp)
                    covariance of the input 
                Ts : float
                    sample period of filter
                N : int
                    number of prediction steps per sample
        '''
        self.n = P0.shape[0]
        self.f = f
        self.Q = Q
        self.P = P0  
        self.xhat = xhat0  
        self.Qu = Qu
        self.N = N
        self.Ts = (Ts / float(N))

    def propagate_model(self, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        '''
            model propagation
                - solve xhatdot=f(xhat,u), and Pdot = AP+PA' + Q + G Qu G' between measurements
            Parameters:
                u: numpy ndarray (p x 1) 
                system inputs            
            Returns:
                xhat and P after one time step ts
        '''
        for i in range(0, self.N):
            # propagate model using Euler 0-method
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, u)
            # compute Jacobian of f with respect to x
            A = self.jacobian(self.f, self.xhat, u)
            # convert to discrete time models
            A_d = np.eye(self.n) + self.Ts * A + ((self.Ts ** 2)/2.) * A @ A
            # compute Jacobian of f with respect to u
            G = self.jacobian_u(self.f, self.xhat, u)
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * (self.Q + G @ self.Qu @ G.T)
        return self.xhat, self.P

    def measurement_update(self, 
                           y: np.ndarray, 
                           u: np.ndarray, 
                           h: Callable, 
                           R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        '''
            Measurement update
                update the state xhat and covarance P when a measurement is received
            Parameters:
                y: numpy ndarray (m x 1) 
                    measurements
                u: numpy ndarray (p x 1) 
                    system inputs
                h : Callable function
                    measurement model yhat=f(xhat, u)
                    x in R^n, u in R^p
                R : numpy ndarray (mxm)
                    covariance of the measurement noise
            Returns:
                xhat and P after measurement update
        '''
        yhat = h(self.xhat, u)  # expected measurement
        # compute Jacobian of h with respect to x        
        H = self.jacobian(h, self.xhat, u)
        S_inv = np.linalg.inv(R + H @ self.P @ H.T) # innovation covariance
        if True: #(y-h).T @ S_inv @ (y-h) < self.threshold:
            L = self.P @ H.T @ S_inv  # Kalman gain
            tmp = np.eye(self.n) - L @ H
            self.P = tmp @ self.P @ tmp.T + L @ R @ L.T # update covariance
            self.xhat = self.xhat + L @ (y - yhat)  # update stae
        return self.xhat, self.P

    def jacobian(self, fun: Callable, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
            Compute jacobian of fun(x,u) with respect to x
                f: R^n x R^p -> R^m
            Parameters:
                x: numpy ndarray (n x 1) 
                u: numpy ndarray (p x 1) 
            Returns:
                J: numpy ndarray (m x n)
        '''
        f = fun(x, u)
        m = f.shape[0]
        n = x.shape[0]
        eps = 0.0001  # deviation
        J = np.zeros((m, n))
        for i in range(0, n):
            x_eps = np.copy(x)
            x_eps[i][0] += eps
            f_eps = fun(x_eps, u)
            df = (f_eps - f) / eps
            J[:, i] = df[:, 0]
        return J

    def jacobian_u(self, fun: Callable, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
            Compute jacobian of fun(x,u) with respect to u
                f: R^n x R^p -> R^m
            Parameters:
                x: numpy ndarray (n x 1) 
                u: numpy ndarray (p x 1) 
            Returns:
                J: numpy ndarray (m x p)
        '''
        f = fun(x, u)
        m = f.shape[0]
        n = u.shape[0]
        eps = 0.0001  # deviation
        J = np.zeros((m, n))
        for i in range(0, n):
            u_eps = np.copy(u)
            u_eps[i][0] += eps
            f_eps = fun(x, u_eps)
            df = (f_eps - f) / eps
            J[:, i] = df[:, 0]
        return J


class KalmanFilterDiscrete:
    ''' 
        base class for discrete-discrete (linear) Kalman filter 
        Assumes continuous dyamamics
            x[k+1] = A x[k] + B u[k] + q(t)
        and discrete measurments
            y[k] = C x[k] + r[k]
        where q and r are zero mean Gaussian with covariances Q and R
    '''
    def __init__(self, 
                 A: np.ndarray, 
                 B: np.ndarray, 
                 C: np.ndarray, 
                 D: np.ndarray, 
                 Q: np.ndarray, 
                 R: np.ndarray, 
                 xhat0: np.ndarray, 
                 P0: np.ndarray):
        '''
            Initialize Ekf class

            Parameters:
                A : numpy ndarray (nxn), system matrix
                B : numpy ndarray (nxm), system matrix
                C : numpy ndarray (pxn), system matrix
                D : numpy ndarray (pxm), system matrix
                Q : numpy ndarray (nxn)
                    covariance of the process noise
                R : numpy ndarray (nxn)
                    covariance of the measurement noise
                xhat0 : numpy ndarray (nxn)
                    initial estimate of the state
                P0 : numpy ndarray (nxn)
                    initial covariance of the estimation error
        '''
        self.n = A.shape[0]
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Q = Q
        self.R = R
        self.xhat = xhat0  
        self.P = P0

    def update(self, y: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        '''
            filter update
                propages xhat and P and performs a measurement update
            Parameters:
                y: numpy ndarray (m x 1) 
                    measurements            
                u: numpy ndarray (p x 1) 
                    system inputs            
            Returns:
                xhat and P after one time step
        '''
        # prediction 
        self.P = self.A.T @ self.P @ self.A + self.Q
        self.xhat = self.A @ self.xhat + self.B @ u
        yhat = self.C @ self.xhat + self.D @ u
        # measure update
        Sinv = np.linalg.inv(self.R + self.C @ self.P @ self.C.T) 
            # inverse of innovation covariance
        L = self.P @ self.C.T @ Sinv # Kalman gain
        tmp = (np.eye(self.n) - L @ self.C)
        self.P = tmp @ self.P @ tmp.T + L @ self.R @ L.T  # Joseph stabilized version
        self.xhat = self.xhat + L @ (y - yhat) # state update
        return self.xhat, self.P


# class KalmanFilterContinuousDiscrete:
#      ''' 
#         Continous-discrete (linear) Kalman filter (EKF)
#         Assumes continuous dyamamics
#             xdot = A x + B u + q(t)
#         and discrete measurments
#             y[k] = C x[k] + D u[k] + r[k]
#         where q and r are zero mean Gaussian with covariances Q and R
#     '''
#     def __init__(self, 
#                  A: np.ndarray, 
#                  B: np.ndarray, 
#                  C: np.ndarray, 
#                  D: np.ndarray, 
#                  Q: np.ndarray, 
#                  R: np.ndarray, 
#                  xhat0: np.ndarray, 
#                  P0: np.ndarray,
#                  Ts : float,
#                  N : int):
#         '''
#             Initialize KF class

#             Parameters:
#                 A : numpy ndarray (nxn), system matrix
#                 B : numpy ndarray (nxm), system matrix
#                 C : numpy ndarray (pxn), system matrix
#                 D : numpy ndarray (pxm), system matrix
#                 Q : numpy ndarray (nxn)
#                     covariance of the process noise
#                 R : numpy ndarray (nxn)
#                     covariance of the measurement noise
#                 xhat0 : numpy ndarray (nxn)
#                     initial estimate of the state
#                 P0 : numpy ndarray (nxn)
#                     initial covariance of the estimation error
#                 Ts : float
#                     sample period of filter
#                 N : int
#                     number of prediction steps per sample
#         '''
#         self.n = A.shape[0]
#         self.A = A
#         self.B = B
#         self.C = C
#         self.D = D
#         self.Q = Q
#         self.R = R
#         self.xhat = xhat0  
#         self.P = P0  
#         self.N = N
#         self.Ts = (Ts / float(N))

#     def update(self, y: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
#         '''
#             filter update
#                 propages xhat and P and performs a measurement update
#             Parameters:
#                 y: numpy ndarray (m x 1) 
#                     measurements            
#                 u: numpy ndarray (p x 1) 
#                     system inputs            
#             Returns:
#                 xhat and P after one time step
#         '''
#         # predication
#         for i in range(0, self.N):
#             self.xhat = self.xhat + self.Ts * (self.A @ self.xhat + self.B @ u)
#             A_d = np.eye(self.n) + self.Ts * self.A + ((self.Ts ** 2)/2.) * self.A @ self.A
#                 # approximate A_d = exp(A*Ts)
#             self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q
#                 # use discrete update to preserve symmetric positive definite
#         # measure update
#         yhat = self.C @ self.xhat + self.D @ u # prediced output   
#         Sinv = np.linalg.inv(self.R + self.C @ self.P @ self.C.T) 
#             # inverse of innovation covariance
#         L = self.P @ self.C.T @ Sinv # Kalman gain
#         tmp = (np.eye(self.n) - L @ self.C)
#         self.P = tmp @ self.P @ tmp.T + L @ self.R @ L.T  # Joseph form (symmetric PD)
#         self.xhat = self.xhat + L @ (y - yhat) # state update
#         return self.xhat, self.P
 