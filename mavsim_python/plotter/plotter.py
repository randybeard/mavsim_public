import pyqtgraph as pg
import time

class Plotter():
    def __init__(self, app, plots_per_row, window_width=1280,
                  window_height=800):
        self._app = app
        self._window = pg.QtWidgets.QMainWindow()
        self._window.resize(window_width,window_height)
        self._plots_per_row = plots_per_row
        if self._plots_per_row < 1:
            self._plots_per_row = 1
        self._layout = pg.QtWidgets.QGridLayout()
        self._num_plots = 0
        self._plot_dict = {}
        self._window_length_list = []
        self._xdata_list = []
        self._ydata_list = []
        self._data_lines_list = []
        self._data_line_labels = []
        self._pen_list = []
        self._widget = pg.QtWidgets.QWidget()
        self._widget.setLayout(self._layout)
        self._window.setCentralWidget(self._widget)
        
    def create_plot_widget(self, plot_id="", xlabel="x_label", ylabel="y_label", 
                        legend=True, background_color="k", window_length=100):
        row = self._num_plots//self._plots_per_row
        col = self._num_plots%self._plots_per_row
        plot_widget = pg.PlotWidget()
        plot_widget.setLabel('left', ylabel)
        plot_widget.setLabel('bottom', xlabel)
        plot_widget.setBackground(background_color)
        if legend == True:
            plot_widget.addLegend()
        self._layout.addWidget(plot_widget,row,col)
        if plot_id == "":
            plot_id = str(self._num_plots)
        self._plot_dict[plot_id] = self._num_plots
        self._window_length_list.append(window_length)
        self._xdata_list.append([])
        self._ydata_list.append([])
        self._data_lines_list.append([])
        self._data_line_labels.append({})
        self._num_plots += 1

    def create_data_set(self, plot_id, data_label, data_color=(255,0,0), data_thickness=15): #add func to change line thickenss
        plot_index = self._plot_dict[plot_id]
        pen = pg.mkPen(color=data_color)
        data_line = self._layout.itemAt(plot_index).widget().plot([], [], 
            name=data_label, width=data_thickness, pen=pen)
        self._data_lines_list[plot_index].append(data_line)
        self._xdata_list[plot_index].append([])
        self._ydata_list[plot_index].append([])
        self._data_line_labels[plot_index][data_label] = len(self._data_lines_list[plot_index]) - 1

    def add_data_point(self, plot_id, data_label, xvalue, yvalue):
        plot_index = self._plot_dict[plot_id]
        dataset_index = self._data_line_labels[plot_index][data_label]
        self._xdata_list[plot_index][dataset_index].append(xvalue)
        self._ydata_list[plot_index][dataset_index].append(yvalue)
        if len(self._xdata_list[plot_index][dataset_index]) > self._window_length_list[plot_index]:
            self._xdata_list[plot_index][dataset_index].pop(0)
            self._ydata_list[plot_index][dataset_index].pop(0)

    def add_data_points(self, plot_id, data_label, xvalues, yvalues):
        plot_index = self._plot_dict[plot_id]
        dataset_index = self._data_line_labels[plot_index][data_label]
        self._xdata_list[plot_index][dataset_index] = self._xdata_list[plot_index][dataset_index] + xvalues
        self._ydata_list[plot_index][dataset_index] = self._ydata_list[plot_index][dataset_index] + yvalues
        len_arr = len(self._xdata_list[plot_index][dataset_index])
        window_length = self._window_length_list[plot_index]
        if len_arr > window_length:
            start_index = len_arr - window_length
            self._xdata_list[plot_index][dataset_index] = self._xdata_list[plot_index][dataset_index][start_index:]
            self._ydata_list[plot_index][dataset_index] = self._ydata_list[plot_index][dataset_index][start_index:]

    def set_plot_data(self,plot_id,data_label,xdata,ydata):
        plot_index = self._plot_dict[plot_id]
        dataset_index = self._data_line_labels[plot_index][data_label]
        self._xdata_list[plot_index][dataset_index] = xdata
        self._ydata_list[plot_index][dataset_index] = ydata
        self._window_length_list[plot_index] = len(self._xdata_list[plot_index][dataset_index])

    def set_window_length(self, plot_id, window_length):
        self._window_length_list[plot_id] = window_length
  
    def update_plots(self):
        for plot_index in range(self._num_plots):
            num_data_sets = len(self._data_lines_list[plot_index])
            for dataset_index in range(num_data_sets):
                self._data_lines_list[plot_index][dataset_index].setData(
                    self._xdata_list[plot_index][dataset_index],
                    self._ydata_list[plot_index][dataset_index])
    
    def process_app(self, sleep_time = 0):
        self._app.processEvents()
        time.sleep(sleep_time)

    def show_window(self, sleep_time = 0):
        self.update_plots()
        self._window.show()
        self.process_app(sleep_time)

    def close_window(self):
        self._window.close()

    def hold_window_until_exit(self):
        self._app.exec()

    def save_image(self,image_name="plotter_image"):
        self._widget.grab().save(image_name+".png")