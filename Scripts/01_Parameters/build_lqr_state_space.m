%% 7-DOF Full Vehicle State Definition
% LQR controller preparation

%% =========================================================
% State Vector Definition
% ==========================================================

% x = [
%   1  z_s
%   2  theta
%   3  phi
%   4  z_u_FL
%   5  z_u_FR
%   6  z_u_RL
%   7  z_u_RR
%   8  z_s_dot
%   9  theta_dot
%  10  phi_dot
%  11  z_u_FL_dot
%  12  z_u_FR_dot
%  13  z_u_RL_dot
%  14  z_u_RR_dot
% ];

n_states = 14;

%% =========================================================
% Control Inputs
% ==========================================================

% u = [
%   F_act_FL
%   F_act_FR
%   F_act_RL
%   F_act_RR
% ];

n_inputs = 4;

%% =========================================================
% Road Disturbances
% ==========================================================

% w = [
%   z_r_FL
%   z_r_FR
%   z_r_RL
%   z_r_RR
% ];

n_disturbances = 4;

%% =========================================================
% State Index Map
% ==========================================================

idx.z_s       = 1;
idx.theta     = 2;
idx.phi       = 3;

idx.z_u_FL    = 4;
idx.z_u_FR    = 5;
idx.z_u_RL    = 6;
idx.z_u_RR    = 7;

idx.z_s_dot   = 8;
idx.theta_dot = 9;
idx.phi_dot   = 10;

idx.z_u_FL_dot = 11;
idx.z_u_FR_dot = 12;
idx.z_u_RL_dot = 13;
idx.z_u_RR_dot = 14;

%% =========================================================
% Display
% ==========================================================

disp('==============================================')
disp('7-DOF LQR STATE DEFINITION')
disp('==============================================')

fprintf('Number of states       = %d\n', n_states);
fprintf('Number of inputs       = %d\n', n_inputs);
fprintf('Number of disturbances = %d\n', n_disturbances);

disp(' ')
disp('State ordering established successfully.')
disp('==============================================')

%% =========================================================
% Build State-Space Matrices
% ==========================================================

A_lqr = zeros(n_states, n_states);
B_lqr = zeros(n_states, n_inputs);
E_lqr = zeros(n_states, n_disturbances);

%% ---------------------------------------------------------
% Kinematic rows
% q_dot = v
% ----------------------------------------------------------

A_lqr(idx.z_s,       idx.z_s_dot)       = 1;
A_lqr(idx.theta,     idx.theta_dot)     = 1;
A_lqr(idx.phi,       idx.phi_dot)       = 1;

A_lqr(idx.z_u_FL,    idx.z_u_FL_dot)    = 1;
A_lqr(idx.z_u_FR,    idx.z_u_FR_dot)    = 1;
A_lqr(idx.z_u_RL,    idx.z_u_RL_dot)    = 1;
A_lqr(idx.z_u_RR,    idx.z_u_RR_dot)    = 1;


%% =========================================================
% Corner Geometry Vectors
% ==========================================================

% z_s_corner = Gq * [z_s theta phi]'
%
% FL = z_s + a*theta + t_f/2*phi
% FR = z_s + a*theta - t_f/2*phi
% RL = z_s - b*theta + t_r/2*phi
% RR = z_s - b*theta - t_r/2*phi

G = [ ...
     1,  a,  t_f/2;
     1,  a, -t_f/2;
     1, -b,  t_r/2;
     1, -b, -t_r/2];


%% =========================================================
% Suspension Parameters Per Corner
% ==========================================================

K_susp = diag([ ...
    k_sf, ...
    k_sf, ...
    k_sr, ...
    k_sr]);

C_susp = diag([ ...
    c_sf, ...
    c_sf, ...
    c_sr, ...
    c_sr]);

M_u = diag([ ...
    m_uf, ...
    m_uf, ...
    m_ur, ...
    m_ur]);


%% =========================================================
% Body Generalized Mass Matrix
% ==========================================================

M_body = diag([ ...
    m_s, ...
    Iyy, ...
    Ixx]);


%% =========================================================
% Force-to-Body Generalized Mapping
% ==========================================================

% Suspension corner forces -> [heave force; pitch moment; roll moment]

H = [ ...
    1,      1,      1,      1;
    a,      a,     -b,     -b;
    t_f/2,  -t_f/2,   t_r/2,  -t_r/2];


%% =========================================================
% Body Acceleration Dynamics
% ==========================================================

% F_susp = -K(G*q_body - q_u) - C(G*qdot_body - qdot_u) + u

A_body_q = ...
    -M_body \ (H * K_susp * G);

A_body_qu = ...
     M_body \ (H * K_susp);

A_body_v = ...
    -M_body \ (H * C_susp * G);

A_body_vu = ...
     M_body \ (H * C_susp);


%% Insert body acceleration terms into A

% q_body = [z_s theta phi]
A_lqr(8:10, 1:3) = A_body_q;

% q_u = [z_u_FL z_u_FR z_u_RL z_u_RR]
A_lqr(8:10, 4:7) = A_body_qu;

% qdot_body
A_lqr(8:10, 8:10) = A_body_v;

% qdot_u
A_lqr(8:10, 11:14) = A_body_vu;


%% =========================================================
% Unsprung-Mass Acceleration Dynamics
% ==========================================================

% m_u*z_u_ddot =
% +K(G*q_body - q_u)
% +C(G*qdot_body - qdot_u)
% -K_tyre(q_u - road)
% -u

K_tyre = k_t * eye(4);

A_u_qbody = ...
    M_u \ (K_susp * G);

A_u_qu = ...
    M_u \ (-K_susp - K_tyre);

A_u_vbody = ...
    M_u \ (C_susp * G);

A_u_vu = ...
    M_u \ (-C_susp);


%% Insert unsprung acceleration terms into A

A_lqr(11:14, 1:3) = A_u_qbody;

A_lqr(11:14, 4:7) = A_u_qu;

A_lqr(11:14, 8:10) = A_u_vbody;

A_lqr(11:14, 11:14) = A_u_vu;


%% =========================================================
% Actuator Input Matrix B
% ==========================================================

% Body sees +u
B_lqr(8:10, :) = ...
    M_body \ H;

% Unsprung masses see -u
B_lqr(11:14, :) = ...
    -inv(M_u);


%% =========================================================
% Road Disturbance Matrix E
% ==========================================================

% Tyre force contribution = K_tyre * road

E_lqr(11:14, :) = ...
    M_u \ K_tyre;


%% =========================================================
% Display Matrix Dimensions
% ==========================================================

disp(' ')
disp('==============================================')
disp('LQR STATE-SPACE MATRICES BUILT')
disp('==============================================')

fprintf('Size of A_lqr = %d x %d\n', ...
    size(A_lqr,1), size(A_lqr,2));

fprintf('Size of B_lqr = %d x %d\n', ...
    size(B_lqr,1), size(B_lqr,2));

fprintf('Size of E_lqr = %d x %d\n', ...
    size(E_lqr,1), size(E_lqr,2));

disp('==============================================')