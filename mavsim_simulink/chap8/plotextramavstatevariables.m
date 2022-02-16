function plotextramavstatevariables(uu,SENSOR)
%
% modified 12/11/2009 - RB
%           5/17/2010 - RB
%
%  plot groundspeed Vg, heading psi, wind North wn, and wind East we

    % process inputs to function
%    pn          = uu(1);             % North position (meters)
%    pe          = uu(2);             % East position (meters)
%    h           = -uu(3);            % altitude (meters)
%    u           = uu(4);             % body velocity along x-axis (meters/s)
%    v           = uu(5);             % body velocity along y-axis (meters/s)
%    w           = uu(6);             % body velocity along z-axis (meters/s)
%    phi         = 180/pi*uu(7);      % roll angle (degrees)   
%    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = uu(9);              % yaw angle (degrees)
%    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
%    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
%    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    Va          = uu(13);            % airspeed (m/s)
%    alpha       = 180/pi*uu(14);     % angle of attack (degrees)
%    beta        = 180/pi*uu(15);     % side slip angle (degrees)
    wn          = uu(16);            % wind in the North direction
    we          = uu(17);            % wind in the East direction
%    wd          = uu(18);            % wind in the Down direction
%    pn_c        = uu(19);            % commanded North position (meters)
%    pe_c        = uu(20);            % commanded East position (meters)
%    h_c         = uu(21);            % commanded altitude (meters)
%    Va_c        = uu(22);            % commanded airspeed (meters/s)
%    alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
%    beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
%    phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
%    theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
%    chi_c       = 180/pi*uu(27);     % commanded course (degrees)
%    p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
%    q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
%    r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
%    pn_hat      = uu(31);            % estimated North position (meters)
%    pe_hat      = uu(32);            % estimated East position (meters)
%    h_hat       = uu(33);            % estimated altitude (meters)
    Va_hat      = uu(34);            % estimated airspeed (meters/s)
%    alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
%    beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
%    phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)   
%    theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
%    chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
%    p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
%    q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
%    r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
    Vg_hat      = uu(43);            % estimated groundspeed
    wn_hat      = uu(44);            % estimated North wind
    we_hat      = uu(45);            % estimated East wind
    psi_hat     = 180/pi*uu(46);     % estimate of heading
    bx_hat      = uu(47);            % estimated x-gyro bias
    by_hat      = uu(48);            % estimated y-gyro bias
    bz_hat      = uu(49);            % estimated z-gyro bias
%    delta_e     = 180/pi*uu(50);     % elevator angle (degrees)
%    delta_a     = 180/pi*uu(51);     % aileron angle (degrees)
%    delta_r     = 180/pi*uu(52);     % rudder angle (degrees)
%    delta_t     = uu(53);            % throttle setting (unitless)
    t           = uu(54);            % simulation time
    
    bx = SENSOR.gyro_x_bias;             % x-gyro bias
    by = SENSOR.gyro_y_bias;             % y-gyro bias
    bz = SENSOR.gyro_z_bias;             % z-gyro bias

    
    % compute true ground speed Vg
    Vg = norm( [Va*cos(psi)+wn; Va*sin(psi)+we] );
    psi = 180/pi*psi;

    % define persistent variables 
    persistent Vg_handle
    persistent psi_handle
    persistent wn_handle
    persistent we_handle
    persistent bx_handle
    persistent by_handle
    persistent bz_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(3), clf

        subplot(4,2,1)
        hold on
        Vg_handle = graph_y_yhat_yd(t, Vg, Vg_hat, 0, 'V_g', []);
        
        subplot(4,2,2)
        hold on
        psi_handle = graph_y_yhat_yd(t, psi, psi_hat, 0, '\psi', []);

        subplot(4,2,3)
        hold on
        wn_handle = graph_y_yhat_yd(t, wn, wn_hat, 0, 'w_n', []);

        subplot(4,2,4)
        hold on
        we_handle = graph_y_yhat_yd(t, we, we_hat, 0, 'w_e', []);

        subplot(4,2,5)
        hold on
        bx_handle = graph_y_yhat_yd(t, bx, bx_hat, 0, 'b_x', []);

        subplot(4,2,6)
        hold on
        by_handle = graph_y_yhat_yd(t, by, by_hat, 0, 'b_y', []);
        
        subplot(4,2,7)
        hold on
        bz_handle = graph_y_yhat_yd(t, bz, bz_hat, 0, 'b_z', []);

    % at every other time step, redraw state variables
    else 
       graph_y_yhat_yd(t, Vg, Vg_hat, 0, 'V_g', Vg_handle);
       graph_y_yhat_yd(t, psi, psi_hat, 0, '\psi', psi_handle);
       graph_y_yhat_yd(t, wn, wn_hat, 0, 'w_n', wn_handle);
       graph_y_yhat_yd(t, we, we_hat, 0, 'w_e', we_handle);
       graph_y_yhat_yd(t, bx, bx_hat, 0, 'b_x', bx_handle);
       graph_y_yhat_yd(t, by, by_hat, 0, 'b_y', by_handle);
       graph_y_yhat_yd(t, bz, bz_hat, 0, 'b_z', bz_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle),
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle),
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

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


