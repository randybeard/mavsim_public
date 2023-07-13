from plotter import Plotter
import random
import numpy as np

p = Plotter(3)
p.show_window()
p.create_plot_widget(plot_id="foo", xlabel="xXx", ylabel="yYy",background_color='w')
p.create_plot_widget(plot_id="fig", xlabel="xXx", ylabel="yYy")
p.create_plot_widget(plot_id="", xlabel="xXx", ylabel="yYy")
p.create_data_set(plot_id="foo",data_label="truth",data_color=(0,255,0),data_thickness=20)
p.create_data_set(plot_id="fig",data_label="tree",data_color=(0,255,0),data_thickness=15)
p.create_plot_widget(plot_id="fi", xlabel="xs", ylabel="ys")
p.create_data_set(plot_id="fi",data_label="estimate",data_color=(0,0,255),data_thickness=10)
p.create_data_set(plot_id="fi",data_label="truth",data_color=(255,0,0),data_thickness=10)
p.set_plot_data("fig","tree",np.linspace(0,50,200).tolist(),[random.randint(0,100) for _ in range(200)])
p.add_data_points("fig","tree", np.linspace(50,60,50).tolist(), np.linspace(0,200,50).tolist())
for i in range(200):
    p.add_data_point("foo","truth",i,random.randint(0,20))
    p.add_data_point("fi","truth",i,random.randint(0,20))
    p.add_data_point("fi","estimate",i,random.randint(0,20))
    # time.sleep(0.1)
    p.update_window(0.01)
    if i == 20:
        p.save_image()
p.hold_window_until_exit()