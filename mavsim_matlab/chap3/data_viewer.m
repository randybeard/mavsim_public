classdef data_viewer < handle
    %
    %    plot MAV states, estimates, and commands
    %
    %--------------------------------
    properties
    	pn_handle
    	pe_handle
    	h_handle
        wn_handle
    	Va_handle
    	alpha_handle
    	beta_handle
        we_handle
    	phi_handle
    	theta_handle
        psi_handle
        Vg_handle
    	chi_handle
    	p_handle
    	q_handle
    	r_handle
        bx_handle
        by_handle
        bz_handle
        plot_initialized
        time
        plot_rate
        time_last_plot
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = data_viewer
            self.pn_handle = [];
            self.pe_handle = [];
            self.h_handle = [];
            self.wn_handle = [];
            self.Va_handle = [];
            self.alpha_handle = [];
            self.beta_handle = [];
            self.we_handle = [];
            self.phi_handle = [];
            self.theta_handle = [];
            self.psi_handle = [];
            self.Vg_handle = [];
            self.chi_handle = [];
            self.p_handle = [];
            self.q_handle = [];
            self.r_handle = [];
            self.bx_handle = [];
            self.by_handle = [];
            self.bz_handle = [];            
            self.time = 0;
            self.plot_rate = .1;
            self.time_last_plot = self.time;
            self.plot_initialized = 0;    
        end
        %---------------------------
        function self=update(self, true, estimated, commanded, Ts)
            
            if self.plot_initialized==0
                figure(2), clf
                subplot(4,4,1)
                    hold on
                    self.pn_handle = self.graph_y_yhat_yd(self.time, true.pn, estimated.pn, commanded.pn, [], 'p_n');
                subplot(4,4,2)
                    hold on
                    self.Va_handle = self.graph_y_yhat_yd(self.time, true.Va, estimated.Va, commanded.Va, [], 'V_a');
                subplot(4,4,3)
                    hold on
                    self.phi_handle = self.graph_y_yhat_yd(self.time, true.phi, estimated.phi, commanded.phi, [], '\phi');
                subplot(4,4,4)
                    hold on
                    self.p_handle = self.graph_y_yhat(self.time, true.p, estimated.p, [], 'p');
                subplot(4,4,5)
                    hold on
                    self.pe_handle = self.graph_y_yhat(self.time, true.pe, estimated.pe, [], 'p_e');
                subplot(4,4,6)
                    hold on
                    self.alpha_handle = self.graph_y_yhat(self.time, true.alpha, estimated.alpha, [], '\alpha');
                subplot(4,4,7)
                    hold on
                    self.theta_handle = self.graph_y_yhat_yd(self.time, true.theta, estimated.theta, commanded.theta, [], '\theta');
                subplot(4,4,8)
                    hold on
                    self.q_handle = self.graph_y_yhat(self.time, true.q, estimated.q, [], 'q');
                subplot(4,4,9)
                    hold on
                    self.h_handle = self.graph_y_yhat_yd(self.time, true.h, estimated.h, commanded.h, [], 'h');
                subplot(4,4,10)
                    hold on
                    self.beta_handle = self.graph_y_yhat(self.time, true.beta, estimated.beta, [], '\beta');
                subplot(4,4,11)
                    hold on
                    self.psi_handle = self.graph_y_yhat(self.time, true.psi, estimated.psi, [], '\psi');
                subplot(4,4,12)
                    hold on
                    self.r_handle = self.graph_y_yhat(self.time, true.r, estimated.r, [], 'r');
                subplot(4,4,13)
                    hold on
                    self.wn_handle = self.graph_y_yhat(self.time, true.wn, estimated.wn, [], 'wind-n');
                subplot(4,4,14)
                    hold on
                    self.Vg_handle = self.graph_y_yhat(self.time, true.Vg, estimated.Vg, [], 'V_g');
                subplot(4,4,15)
                    hold on
                    self.chi_handle = self.graph_y_yhat_yd(self.time, true.chi, estimated.chi, commanded.chi, [], '\chi');
                subplot(4,4,16)
                    hold on
                    self.bx_handle = self.graph_y_yhat(self.time, true.bx, estimated.bx, [], 'Bias');
                    self.by_handle = self.graph_y_yhat(self.time, true.by, estimated.by, [], []);
                    self.bz_handle = self.graph_y_yhat(self.time, true.bz, estimated.bz, [], []);
                self.plot_initialized = 1;
            else
                if self.time >= self.time_last_plot + self.plot_rate
                    self.graph_y_yhat(self.time, true.pn, estimated.pn, self.pn_handle);
                    self.graph_y_yhat(self.time, true.pe, estimated.pe, self.pe_handle);
                    self.graph_y_yhat_yd(self.time, true.h, estimated.h, commanded.h, self.h_handle);
                    self.graph_y_yhat(self.time, true.wn, estimated.wn, self.wn_handle);
                    self.graph_y_yhat_yd(self.time, true.Va, estimated.Va, commanded.Va, self.Va_handle);
                    self.graph_y_yhat(self.time, true.alpha, estimated.alpha, self.alpha_handle);
                    self.graph_y_yhat(self.time, true.beta, estimated.beta, self.beta_handle);
                    self.graph_y_yhat(self.time, true.Vg, estimated.Vg, self.Vg_handle);
                    self.graph_y_yhat_yd(self.time, true.phi, estimated.phi, commanded.phi, self.phi_handle);
                    self.graph_y_yhat_yd(self.time, true.theta, estimated.theta, commanded.theta, self.theta_handle);
                    self.graph_y_yhat(self.time, true.psi, estimated.psi, self.psi_handle);
                    self.graph_y_yhat_yd(self.time, true.chi, estimated.chi, commanded.chi, self.chi_handle);
                    self.graph_y_yhat(self.time, true.p, estimated.p, self.p_handle);
                    self.graph_y_yhat(self.time, true.q, estimated.q, self.q_handle);
                    self.graph_y_yhat(self.time, true.r, estimated.r, self.r_handle);
                    self.graph_y_yhat(self.time, true.bx, estimated.bx, self.bx_handle);
                    self.graph_y_yhat(self.time, true.by, estimated.by, self.by_handle);
                    self.graph_y_yhat(self.time, true.bz, estimated.bz, self.bz_handle);
                    self.time_last_plot = self.time;
                end
                self.time = self.time + Ts;
            end
        end
        %---------------------------
        function handle = graph_y_yhat_yd(self, t, y, yhat, yd, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,y,'b');
                handle(2)   = plot(t,yhat,'g--');
                handle(3)   = plot(t,yd,'r-.');
                ylabel(lab)
                set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
                set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
                set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
                set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
                set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
                %drawnow
            end
        end
        %---------------------------
        function handle = graph_y_yhat(self, t, y, yhat, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,y,'b');
                handle(2)   = plot(t,yhat,'g--');
                ylabel(lab)
                set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
                set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
                set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
                %drawnow
            end
        end
        
    end
end