"""
data_viewer

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
        2/27/2020 - RWB
        1/19/2023 - DLC
"""
from plotter.plotter import Plotter
import numpy as np
from tools.wrap import wrap

class DataViewer:
    def __init__(self, app,  dt = 0.01,
                 time_window_length = 30, # number of data points plotted at a time
                 plot_period = 0.2, # time interval between a plot update
                 data_recording_period = 0.1): # time interval between recording a data update
        self._dt = dt
        self._data_window_length= time_window_length/data_recording_period
        self._update_counter = 0
        self._plots_per_row = 4
        self._plotter = Plotter(app=app, plots_per_row=self._plots_per_row)  # plot last time_window seconds of data
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0

        #define colors
        truth_color = (0,255,0)
        truth_color_2 = (160,202,111)
        truth_color_3 = (124,230,167)
        estimate_color = (255,0,0)
        estimate_color_2 = (255,150,150)
        estimate_color_3 = (255,154,111)
        control_color = (0,0,255)

        # define first row
        self._plotter.create_plot_widget(plot_id='pn', xlabel='Time (s)', ylabel='pn (m)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='pe', xlabel='Time (s)', ylabel='pe (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='h', xlabel='Time (s)', ylabel='h (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='wind', xlabel='Time (s)', ylabel='wind (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="pn", data_label="pn", data_color=truth_color)
        self._plotter.create_data_set(plot_id="pn", data_label="pn_e", data_color=estimate_color) 
        self._plotter.create_data_set(plot_id="pe", data_label="pe", data_color=truth_color)
        self._plotter.create_data_set(plot_id="pe", data_label="pe_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="h", data_label="h", data_color=truth_color)
        self._plotter.create_data_set(plot_id="h", data_label="h_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="h", data_label="h_c", data_color=control_color)
        self._plotter.create_data_set(plot_id="wind", data_label="wn", data_color=truth_color)
        self._plotter.create_data_set(plot_id="wind", data_label="wn_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="wind", data_label="we", data_color=truth_color_2)
        self._plotter.create_data_set(plot_id="wind", data_label="we_e", data_color=estimate_color_2)

        # define second row
        self._plotter.create_plot_widget(plot_id='Va', xlabel='Time (s)', ylabel='Va (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='alpha', xlabel='Time (s)', ylabel='alpha (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='beta', xlabel='Time (s)', ylabel='beta (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='Vg', xlabel='Time (s)', ylabel='Vg (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="Va", data_label="Va", data_color=truth_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_c", data_color=control_color)
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha", data_color=truth_color)
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="beta", data_label="beta", data_color=truth_color)
        self._plotter.create_data_set(plot_id="beta", data_label="beta_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="Vg", data_label="Vg", data_color=truth_color)
        self._plotter.create_data_set(plot_id="Vg", data_label="Vg_e", data_color=estimate_color)
        
        # define third row
        self._plotter.create_plot_widget(plot_id='phi', xlabel='Time (s)', ylabel='phi (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='theta', xlabel='Time (s)', ylabel='theta (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='psi', xlabel='Time (s)', ylabel='psi (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='chi', xlabel='Time (s)', ylabel='chi (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="phi", data_label="phi", data_color=truth_color)
        self._plotter.create_data_set(plot_id="phi", data_label="phi_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="phi", data_label="phi_c", data_color=control_color)
        self._plotter.create_data_set(plot_id="theta", data_label="theta", data_color=truth_color)
        self._plotter.create_data_set(plot_id="theta", data_label="theta_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="theta", data_label="theta_c", data_color=control_color)
        self._plotter.create_data_set(plot_id="psi", data_label="psi", data_color=truth_color)
        self._plotter.create_data_set(plot_id="psi", data_label="psi_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="psi", data_label="psi_c", data_color=control_color)
        self._plotter.create_data_set(plot_id="chi", data_label="chi", data_color=truth_color)
        self._plotter.create_data_set(plot_id="chi", data_label="chi_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="chi", data_label="chi_c", data_color=control_color)

        # define fourth row
        self._plotter.create_plot_widget(plot_id='p', xlabel='Time (s)', ylabel='p (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='q', xlabel='Time (s)', ylabel='q (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='r', xlabel='Time (s)', ylabel='r (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='bias', xlabel='Time (s)', ylabel='bias (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="p", data_label="p", data_color=truth_color)
        self._plotter.create_data_set(plot_id="p", data_label="p_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="q", data_label="q", data_color=truth_color)
        self._plotter.create_data_set(plot_id="q", data_label="q_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="r", data_label="r", data_color=truth_color)
        self._plotter.create_data_set(plot_id="r", data_label="r_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="bias", data_label="bx", data_color=truth_color)
        self._plotter.create_data_set(plot_id="bias", data_label="bx_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="bias", data_label="by", data_color=truth_color_2)
        self._plotter.create_data_set(plot_id="bias", data_label="by_e", data_color=estimate_color_2)
        self._plotter.create_data_set(plot_id="bias", data_label="bz", data_color=truth_color_3)
        self._plotter.create_data_set(plot_id="bias", data_label="bz_e", data_color=estimate_color_3)

        # define fifth row
        self._plotter.create_plot_widget(plot_id='delta_e', xlabel='Time (s)', ylabel='delta_e (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_a', xlabel='Time (s)', ylabel='delta_a (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_r', xlabel='Time (s)', ylabel='delta_r (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_t', xlabel='Time (s)', ylabel='delta_t (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="delta_e", data_label="delta_e", data_color=control_color)
        self._plotter.create_data_set(plot_id="delta_a", data_label="delta_a", data_color=control_color)
        self._plotter.create_data_set(plot_id="delta_r", data_label="delta_r", data_color=control_color)
        self._plotter.create_data_set(plot_id="delta_t", data_label="delta_t", data_color=control_color)
        self._plotter.show_window()

    def update(self, true_state, estimated_state, commanded_state, delta):
        if self._data_recording_delay >= self._data_recording_period:
            self.__update_data(true_state, estimated_state, commanded_state, delta, self._time)
            self._data_recording_delay = 0
        if self._plot_delay >= self._plot_period:
            self.__update_plot()
            self._plot_delay = 0
        self._plot_delay += self._dt
        self._data_recording_delay += self._dt
        self._time += self._dt
        
    def __update_data(self, true_state, estimated_state, commanded_state, delta, t):
        #add the commanded state data
        if commanded_state != None:
            self._plotter.add_data_point(plot_id='h', data_label='h_c', xvalue=t, yvalue=commanded_state.altitude)
            self._plotter.add_data_point(plot_id='Va', data_label='Va_c', xvalue=t, yvalue=commanded_state.Va)
            self._plotter.add_data_point(plot_id='phi', data_label='phi_c', xvalue=t, yvalue=self.__rad_to_deg(commanded_state.phi))
            self._plotter.add_data_point(plot_id='theta', data_label='theta_c', xvalue=t, yvalue=self.__rad_to_deg(commanded_state.theta))
            self._plotter.add_data_point(plot_id='chi', data_label='chi_c', xvalue=t, yvalue=self.__rad_to_deg(commanded_state.chi))
        #add the true state data
        if true_state != None:
            self._plotter.add_data_point(plot_id='pn', data_label='pn', xvalue=t, yvalue=true_state.north)
            self._plotter.add_data_point(plot_id='pe', data_label='pe', xvalue=t, yvalue=true_state.east)
            self._plotter.add_data_point(plot_id='h', data_label='h', xvalue=t, yvalue=true_state.altitude)
            self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=t, yvalue=true_state.Va)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha', xvalue=t, yvalue=true_state.alpha)
            self._plotter.add_data_point(plot_id='beta', data_label='beta', xvalue=t, yvalue=true_state.beta)
            self._plotter.add_data_point(plot_id='phi', data_label='phi', xvalue=t, yvalue=self.__rad_to_deg(true_state.phi))
            self._plotter.add_data_point(plot_id='theta', data_label='theta', xvalue=t, yvalue=self.__rad_to_deg(true_state.theta))
            self._plotter.add_data_point(plot_id='psi', data_label='psi', xvalue=t, yvalue=self.__rad_to_deg(true_state.psi))
            self._plotter.add_data_point(plot_id='chi', data_label='chi', xvalue=t, yvalue=self.__rad_to_deg(true_state.chi))
            self._plotter.add_data_point(plot_id='p', data_label='p', xvalue=t, yvalue=self.__rad_to_deg(true_state.p))
            self._plotter.add_data_point(plot_id='q', data_label='q', xvalue=t, yvalue=self.__rad_to_deg(true_state.q))
            self._plotter.add_data_point(plot_id='r', data_label='r', xvalue=t, yvalue=self.__rad_to_deg(true_state.r))
            self._plotter.add_data_point(plot_id='Vg', data_label='Vg', xvalue=t, yvalue=true_state.Vg)
            self._plotter.add_data_point(plot_id='wind', data_label='wn', xvalue=t, yvalue=true_state.wn)
            self._plotter.add_data_point(plot_id='wind', data_label='we', xvalue=t, yvalue=true_state.we)
            self._plotter.add_data_point(plot_id='bias', data_label='bx', xvalue=t, yvalue=self.__rad_to_deg(true_state.bx))
            self._plotter.add_data_point(plot_id='bias', data_label='by', xvalue=t, yvalue=self.__rad_to_deg(true_state.by))
            self._plotter.add_data_point(plot_id='bias', data_label='bz', xvalue=t, yvalue=self.__rad_to_deg(true_state.bz))
        #add the estimated state data
        if estimated_state != None:
            self._plotter.add_data_point(plot_id='pn', data_label='pn_e', xvalue=t, yvalue=estimated_state.north)
            self._plotter.add_data_point(plot_id='pe', data_label='pe_e', xvalue=t, yvalue=estimated_state.east)
            self._plotter.add_data_point(plot_id='h', data_label='h_e', xvalue=t, yvalue=estimated_state.altitude)
            self._plotter.add_data_point(plot_id='Va', data_label='Va_e', xvalue=t, yvalue=estimated_state.Va)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha_e', xvalue=t, yvalue=estimated_state.alpha)
            self._plotter.add_data_point(plot_id='beta', data_label='beta_e', xvalue=t, yvalue=estimated_state.beta)
            self._plotter.add_data_point(plot_id='phi', data_label='phi_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.phi))
            self._plotter.add_data_point(plot_id='theta', data_label='theta_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.theta))
            self._plotter.add_data_point(plot_id='psi', data_label='psi_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.psi))
            self._plotter.add_data_point(plot_id='chi', data_label='chi_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.chi))
            self._plotter.add_data_point(plot_id='p', data_label='p_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.p))
            self._plotter.add_data_point(plot_id='q', data_label='q_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.q))
            self._plotter.add_data_point(plot_id='r', data_label='r_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.r))
            self._plotter.add_data_point(plot_id='Vg', data_label='Vg_e', xvalue=t, yvalue=estimated_state.Vg)
            self._plotter.add_data_point(plot_id='wind', data_label='wn_e', xvalue=t, yvalue=estimated_state.wn)
            self._plotter.add_data_point(plot_id='wind', data_label='we_e', xvalue=t, yvalue=estimated_state.we)
            self._plotter.add_data_point(plot_id='bias', data_label='bx_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.bx))
            self._plotter.add_data_point(plot_id='bias', data_label='by_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.by))
            self._plotter.add_data_point(plot_id='bias', data_label='bz_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.bz))
        #add control data
        if delta != None:
            self._plotter.add_data_point(plot_id='delta_e', data_label='delta_e', xvalue=t, yvalue=self.__rad_to_deg(delta.elevator))
            self._plotter.add_data_point(plot_id='delta_a', data_label='delta_a', xvalue=t, yvalue=self.__rad_to_deg(delta.aileron))
            self._plotter.add_data_point(plot_id='delta_r', data_label='delta_r', xvalue=t, yvalue=self.__rad_to_deg(delta.rudder))
            self._plotter.add_data_point(plot_id='delta_t', data_label='delta_t', xvalue=t, yvalue=self.__rad_to_deg(delta.throttle))

    def process_app(self):
        self._plotter.process_app(0)

    def __update_plot(self):
        self._plotter.update_plots()

    def close_data_viewer(self):
        self._plotter.close_window()

    def save_plot_image(self, plot_name):
        self._plotter.save_image(plot_name)

    def __rad_to_deg(self, radians):
        rad = wrap(radians,0)
        return rad*180/np.pi


