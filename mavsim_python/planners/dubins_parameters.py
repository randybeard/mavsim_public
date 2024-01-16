# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB
#         3/30/2022 - RWB

import numpy as np


class DubinsParameters:

    def update(self, ps, chis, pe, chie, R):
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.compute_parameters()

    def compute_parameters(self):
        ps = self.p_s
        pe = self.p_e
        chis = self.chi_s
        chie = self.chi_e
        R = self.radius
        ell = np.linalg.norm(ps[0:2] - pe[0:2])

        ##### TODO #####
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
        else:
            # compute start and end circles
            crs = 0
            cls = 0
            cre = 0
            cle = 0

            # compute L1
            L1 = 0

            # compute L2
            L2 = 0

            # compute L3
            L3 = 0

            # compute L4
            L4 = 0

            # L is the minimum distance
            L = np.min([L1, L2, L3, L4])
            min_idx = np.argmin([L1, L2, L3, L4])

            if min_idx == 0:
                pass
            elif min_idx == 1:
                pass
            elif min_idx == 2:
                pass
            elif min_idx == 3:
                pass
            self.length = 0
            self.center_s = 0
            self.dir_s = 0
            self.center_e = 0
            self.dir_e = 0
            self.r1 = 0
            self.n1 = 0
            self.r2 = 0
            self.r3 = 0
            self.n3 = 0

    def compute_points(self):
        ##### TODO ##### - uncomment lines and remove last line
        # Del = 0.1  # distance between point

        # # points along start circle
        # th1 = np.arctan2(self.p_s.item(1) - self.center_s.item(1),
        #                  self.p_s.item(0) - self.center_s.item(0))
        # th1 = mod(th1)
        # th2 = np.arctan2(self.r1.item(1) - self.center_s.item(1),
        #                  self.r1.item(0) - self.center_s.item(0))
        # th2 = mod(th2)
        # th = th1
        # theta_list = [th]
        # if self.dir_s > 0:
        #     if th1 >= th2:
        #         while th < th2 + 2*np.pi - Del:
        #             th += Del
        #             theta_list.append(th)
        #     else:
        #         while th < th2 - Del:
        #             th += Del
        #             theta_list.append(th)
        # else:
        #     if th1 <= th2:
        #         while th > th2 - 2*np.pi + Del:
        #             th -= Del
        #             theta_list.append(th)
        #     else:
        #         while th > th2 + Del:
        #             th -= Del
        #             theta_list.append(th)

        # points = np.array([[self.center_s.item(0) + self.radius * np.cos(theta_list[0]),
        #                     self.center_s.item(1) + self.radius * np.sin(theta_list[0]),
        #                     self.center_s.item(2)]])
        # for angle in theta_list:
        #     new_point = np.array([[self.center_s.item(0) + self.radius * np.cos(angle),
        #                            self.center_s.item(1) + self.radius * np.sin(angle),
        #                            self.center_s.item(2)]])
        #     points = np.concatenate((points, new_point), axis=0)

        # # points along straight line
        # sig = 0
        # while sig <= 1:
        #     new_point = np.array([[(1 - sig) * self.r1.item(0) + sig * self.r2.item(0),
        #                            (1 - sig) * self.r1.item(1) + sig * self.r2.item(1),
        #                            (1 - sig) * self.r1.item(2) + sig * self.r2.item(2)]])
        #     points = np.concatenate((points, new_point), axis=0)
        #     sig += Del

        # # points along end circle
        # th2 = np.arctan2(self.p_e.item(1) - self.center_e.item(1),
        #                  self.p_e.item(0) - self.center_e.item(0))
        # th2 = mod(th2)
        # th1 = np.arctan2(self.r2.item(1) - self.center_e.item(1),
        #                  self.r2.item(0) - self.center_e.item(0))
        # th1 = mod(th1)
        # th = th1
        # theta_list = [th]
        # if self.dir_e > 0:
        #     if th1 >= th2:
        #         while th < th2 + 2 * np.pi - Del:
        #             th += Del
        #             theta_list.append(th)
        #     else:
        #         while th < th2 - Del:
        #             th += Del
        #             theta_list.append(th)
        # else:
        #     if th1 <= th2:
        #         while th > th2 - 2 * np.pi + Del:
        #             th -= Del
        #             theta_list.append(th)
        #     else:
        #         while th > th2 + Del:
        #             th -= Del
        #             theta_list.append(th)
        # for angle in theta_list:
        #     new_point = np.array([[self.center_e.item(0) + self.radius * np.cos(angle),
        #                            self.center_e.item(1) + self.radius * np.sin(angle),
        #                            self.center_e.item(2)]])
        #     points = np.concatenate((points, new_point), axis=0)
        points = np.zeros((5,3))
        return points


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


