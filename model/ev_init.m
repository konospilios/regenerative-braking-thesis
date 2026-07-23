%% ev_init.m  (run this before sim)
% Battery
Vbatt      = 400;          % V (your pack)
Rbatt      = 0.05;         % ohm (small series resistance for realism)

% DC motor (SI units)
Rm         = 0.2;          % armature resistance [ohm]
Lm         = 500e-6;       % armature inductance [H]
Kt         = 0.08;         % torque constant [N*m/A]
Ke         = Kt;           % back-EMF constant [V*s/rad] (SI equality)
Jm         = 2e-4;         % rotor inertia [kg*m^2]
Bm         = 5e-5;         % viscous friction [N*m*s/rad]

% Gear & load
Ng         = 5;            % gear ratio (motor:load). Define sign conv.
eta_g      = 0.98;         % gear efficiency
Jload      = 0.02;         % load inertia [kg*m^2] at load side
Bload      = 5e-3;         % load viscous friction [N*m*s/rad]

% Initial targets (used by Variable Viewer)
w0_load    = 0;            % initial load speed [rad/s]
i0         = 0;            % initial armature current [A]

% Derived convenience (optional)
J_equiv = Jm + (Jload*(Ng^2)/eta_g);    % reflected inertia to motor side (approx.)
B_equiv = Bm + (Bload*(Ng^2)/eta_g);    % reflected damping to motor side