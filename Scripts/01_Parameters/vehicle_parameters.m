%% 7-DOF Active Suspension Race Car Parameters
% Project:
% 7-DOF Active Suspension Modelling and Control for a Race Vehicle
%
% Based on the validated parameters used in the previous
% Suspension Shaker Rig assignment.

clc;

%% =========================================================
%  Gravity
% ==========================================================

g = 9.81;              % Gravitational acceleration [m/s^2]


%% =========================================================
%  Vehicle Dimensions
% ==========================================================

L = 3.50;              % Wheelbase [m]

a = 1.80;              % Distance from CG to front axle [m]
b = L - a;             % Distance from CG to rear axle [m]

t_f = 1.75;            % Front track width [m]
t_r = 1.85;            % Rear track width [m]

h_cg = 0.20;           % Centre of gravity height [m]

R_wheel = 0.375;       % Wheel radius [m]


%% =========================================================
%  Roll Centre Heights
% ==========================================================

h_rcf = 0.04;          % Front roll centre height [m]
h_rcr = 0.06;          % Rear roll centre height [m]

h_rc = 0.05;           % Approximate overall roll centre height [m]


%% =========================================================
%  Unsprung Mass CG Heights
% ==========================================================

h_USMf = R_wheel;      % Front unsprung mass CG height [m]
h_USMr = R_wheel;      % Rear unsprung mass CG height [m]


%% =========================================================
%  Vehicle Masses
% ==========================================================

m = 850;               % Total vehicle mass [kg]

m_uf = 15;             % Front unsprung mass per corner [kg]
m_ur = 17.5;           % Rear unsprung mass per corner [kg]

% Sprung mass
m_s = m - 2*m_uf - 2*m_ur;

% Result:
% m_s = 785 kg


%% =========================================================
%  Vehicle Inertia
% ==========================================================

Ixx = 300;             % Roll moment of inertia [kg*m^2]

Iyy = 1350;            % Pitch moment of inertia [kg*m^2]


%% =========================================================
%  Aerodynamic Parameters
% ==========================================================

A = 1.25;              % Frontal area [m^2]

Cz = 1.00;             % Downforce coefficient

rho = 1.225;           % Air density [kg/m^3]


%% =========================================================
%  Front Suspension
% ==========================================================

k_sf = 67500;          % Front spring stiffness per corner [N/m]

c_sf = 1400;           % Front damping coefficient per corner [N*s/m]

k_ARBf = 5000;         % Front anti-roll bar stiffness [Nm/rad]


%% =========================================================
%  Rear Suspension
% ==========================================================

k_sr = 100000;         % Rear spring stiffness per corner [N/m]

c_sr = 1800;           % Rear damping coefficient per corner [N*s/m]

k_ARBr = 1000;         % Rear anti-roll bar stiffness [Nm/rad]


%% =========================================================
%  Tyre Parameters
% ==========================================================

k_t = 400000;          % Tyre vertical stiffness [N/m]


%% =========================================================
%  Static Axle Loads
% ==========================================================

% Sprung mass load distribution

F_front_sprung = m_s * g * (b / L);

F_rear_sprung = m_s * g * (a / L);


%% =========================================================
%  Static Load Per Corner
% ==========================================================

F_FL_static = F_front_sprung / 2 + m_uf * g;

F_FR_static = F_FL_static;

F_RL_static = F_rear_sprung / 2 + m_ur * g;

F_RR_static = F_RL_static;


%% =========================================================
%  Tyre Static Deflections
% ==========================================================

z0t_FL = F_FL_static / k_t;

z0t_FR = z0t_FL;

z0t_RL = F_RL_static / k_t;

z0t_RR = z0t_RL;


%% =========================================================
%  Initial Active Suspension Forces
% =========================================================
% Passive baseline:
% Active actuator force is initially zero.

F_act_FL = 0;          % Front-left actuator force [N]
F_act_FR = 0;          % Front-right actuator force [N]

F_act_RL = 0;          % Rear-left actuator force [N]
F_act_RR = 0;          % Rear-right actuator force [N]


%% =========================================================
%  Initial Road Inputs
% =========================================================
% Flat road for initial passive model verification.

z_r_FL = 0;            % Front-left road displacement [m]
z_r_FR = 0;            % Front-right road displacement [m]

z_r_RL = 0;            % Rear-left road displacement [m]
z_r_RR = 0;            % Rear-right road displacement [m]


%% =========================================================
%  Initial Vehicle States
% ==========================================================

% Sprung mass

z_s0 = 0;              % Initial heave displacement [m]
z_s_dot0 = 0;          % Initial heave velocity [m/s]

theta0 = 0;            % Initial pitch angle [rad]
theta_dot0 = 0;        % Initial pitch rate [rad/s]

phi0 = 0;              % Initial roll angle [rad]
phi_dot0 = 0;          % Initial roll rate [rad/s]


% Unsprung masses

z_u_FL0 = 0;
z_u_FR0 = 0;
z_u_RL0 = 0;
z_u_RR0 = 0;

z_u_FL_dot0 = 0;
z_u_FR_dot0 = 0;
z_u_RL_dot0 = 0;
z_u_RR_dot0 = 0;

%% =========================================================
%  Road Excitation - Symmetric Bump Test
% ==========================================================

V = 20;                     % Vehicle speed [m/s]

bump_height = 0.025;        % Bump height [m]
bump_length = 0.50;         % Bump longitudinal length [m]

bump_duration = bump_length / V;

t_bump_front = 1.0;         % Time front axle reaches bump [s]

rear_delay = L / V;

t_bump_rear = t_bump_front + rear_delay;

%% =========================================================
% Frequency Sweep Test
% ==========================================================

sweep_amplitude = 0.005;     % Road amplitude [m] = 5 mm

sweep_f_start = 0.5;         % Start frequency [Hz]
sweep_f_end   = 25;          % End frequency [Hz]

sweep_time = 20;             % Sweep duration [s]

Fs = 1000;                   % Simulation sampling frequency [Hz]
Ts = 1/Fs;                   % Sample time [s]

%% =========================================================
% Active Suspension Actuator Limits
% ==========================================================

F_act_max = 3000;      % Maximum actuator force per corner [N]

active_mode = 1;       % 0 = Passive, 1 = Active

%% =========================================================
% Skyhook Controller - Initial Gain
% ==========================================================

C_sky = 6000;     % Skyhook damping gain [N*s/m]

%% =========================================================
% Asymmetric Kerb Test
% ==========================================================

kerb_height = 0.030;          % Kerb height [m]
kerb_length = 0.60;           % Kerb length [m]

kerb_duration = kerb_length / V;

t_kerb_front = 1.0;
t_kerb_rear  = t_kerb_front + rear_delay;


%% =========================================================
%  Display Main Parameters
% ==========================================================

disp('==============================================')
disp('7-DOF Active Suspension Vehicle Parameters')
disp('==============================================')

fprintf('Total Vehicle Mass       = %.1f kg\n', m);
fprintf('Sprung Mass              = %.1f kg\n', m_s);

fprintf('Front Unsprung Mass      = %.1f kg per corner\n', m_uf);
fprintf('Rear Unsprung Mass       = %.1f kg per corner\n', m_ur);

fprintf('Front Spring Stiffness   = %.0f N/m\n', k_sf);
fprintf('Rear Spring Stiffness    = %.0f N/m\n', k_sr);

fprintf('Front Damping            = %.0f Ns/m\n', c_sf);
fprintf('Rear Damping             = %.0f Ns/m\n', c_sr);

fprintf('Tyre Stiffness           = %.0f N/m\n', k_t);

disp('==============================================')


%%
controller_mode = 1;   

