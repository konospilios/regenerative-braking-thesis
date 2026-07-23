% Init:
close all

Simulation_time = 2; % s

Motor_voltage = 300; % V
Motor_no_load_speed = 8000; % RPM
Motor_rated_speed = 5000; % RPM
Motor_rated_load = 50000; % W

Gear_ratio = 100; %
Vehicle_mass = 2000; % Kg / Assuming a wheel radius of 350mm
Damping_coef = 50; % N*m*s/rad

% Plot: 
LineWidth = 3;


out = sim('Simple_powertrain_test1');


figure
t = tiledlayout(5,1);   % rows, columns
title(t,"The best Title")



% First plot
ax1 = nexttile;
plot(ax1,out.tout,out.Veh_Vel.signals.values, 'LineWidth', LineWidth)
hold(ax1,'on')
plot(ax1,out.tout,out.Veh_ref.signals.values, 'LineWidth', LineWidth)
plot(ax1,out.tout,out.Veh_ref_err.signals.values, 'LineWidth', LineWidth)
ylabel(ax1,'Velocity [m/s]')
xlabel(ax1,'Time [s]')
legend(ax1,{'Vehicle velocity','Reference velocity',...
    'Reference error'}, 'Location','best')
grid(ax1, 'on')

% Second plot
ax2 = nexttile;
plot(ax2,out.tout,out.Veh_ref_err.signals.values, 'LineWidth', LineWidth)
hold(ax2,'on')
plot(ax2,out.tout,out.Driver_acc_cmd.signals.values, 'LineWidth', LineWidth)
plot(ax2,out.tout,out.Driver_brake_cmd1.signals.values, 'LineWidth', LineWidth)
ylabel(ax2,'Values')
xlabel(ax2,'Time [s]')
legend (ax2, {'Reference error [m/s]', 'Acc cmd', 'Brake cmd'}, 'Location','best')
grid(ax2, 'on')

% Third plot
ax3 = nexttile;
plot(ax3,out.tout,out.Btry_SOC.signals.values*100, 'LineWidth', LineWidth)
hold(ax3,'on')

ylabel(ax3, 'SOC [%]')
xlabel(ax3,'Time [s]')
legend (ax3, {'Battery State Of Charge'}, 'Location','best')
grid(ax3, 'on')

% Fith plot
ax4 = nexttile;
plot(ax4,out.tout,out.Btry_V.signals.values, 'LineWidth', LineWidth)
ylabel(ax4,'Voltage [V]')
xlabel(ax4,'Time [s]')
legend (ax4, {'Battery Voltage'}, 'Location','best')
grid(ax4, 'on')



% Fifth plot
ax5 = nexttile;
plot(ax5,out.tout,out.Btry_I_in.signals.values, 'LineWidth', LineWidth)
hold(ax5,'on')
ylabel(ax5,'Current [A]')
xlabel(ax5,'Time [s]')
legend (ax5, {'Battery Current'}, 'Location','best')
grid(ax5, 'on')




