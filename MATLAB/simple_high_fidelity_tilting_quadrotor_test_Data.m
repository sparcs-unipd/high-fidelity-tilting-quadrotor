% Initialization data for the hight fidelity tilting quadrotor model

skew = @(x) [
    0 -x(3) x(2)
    x(3) 0 -x(1)
    -x(2) x(1) 0
    ];

N = 4;

% propeller data
prop.m = 5e-3;      % propeller mass [kg]
prop.R = 80e-3;     % propeller Radius [m]
prop.h = 10e-3;     % propeller height [m]
% propeller inertia matrix [kg*m^2]
I_prop = diag([ ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/2*prop.m*prop.R^2]);

rotor.m = 50e-3;    % rotor mass [kg]
rotor.R = 20e-3;    % rotor Radius [m]
rotor.h = 10e-3;    % rotor height [m]
% rotor inertia matrix [kg*m^2]
I_rotor = diag([ ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/2*rotor.m*rotor.R^2]);

I_i = I_prop+I_rotor;   % total rotor+propeller inertia [Kg*m^2]

stator.m = 100e-3;      % stator mass [kg]

% main body data
body.m = 2;             % body mass [kg]
body.R = 100e-3;        % body Radius [m]
body.h = 50e-3;         % body height [m]
I_b = diag([ ...        % body inertia [kg*m^2]
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/2*body.m*body.R^2]);
m_b = body.m + N*(stator.m+rotor.m+prop.m);       % body mass [Kg]

prop.w_max_rpm = 10000;                 % propeller maximum rate [rpm]
prop.w_max = prop.w_max_rpm * 2*pi/60;  % propeller maximum rate [rad/s]

prop.max_thrust = 3*m_b*9.81/4;         % propeller maximum thrust [N]           

k_t = prop.max_thrust / prop.w_max.^2;  % thrust coefficient [N/s^2]
k_m = k_t * 0.01;                       % propeller drag coefficient [Nm/s^2]

% motor torque needed to reach the maximum speed.
% it has to counter the drag torque at maximum speed
prop.max_drag = k_m * prop.w_max.^2;

% propeller spinning convention
% 1 if positive velocity (CCW) gives positive thrust
% -1 if negative velocity (CW) gives positive thrust
c_direction = [1 1 -1 -1];

% rotor position in body frame [m]
%   3(CW)   front    1(CCW)
%   left      +      right
%   2(CCW)   back    4(CW)
p_mot = ...         % rotor position w.r.t. body
    [ 0.2 -0.2 0;
     -0.2  0.2 0;
      0.2  0.2 0;
     -0.2 -0.2 0];
theta = atan2(p_mot(:,2),p_mot(:,1));   % arm direction angle
beta = theta;       % propellers tilting plane yaw angle [rad]

% final body inertia when the masses of motors and propellers are
% considered
for k=1:4
    I_b = I_b - (stator.m+rotor.m+prop.m)*skew(p_mot(k,:))^2;
end

% HOVER DATA
required_hover_thrust = m_b * 9.81;
w_bar_hover_planar = sqrt(required_hover_thrust/4/k_t);

% planar control allocation matrix
% torque allocation matrix
M = cross(p_mot.',kron([0;0;1],c_direction)) * k_t +...
    kron([0;0;1],[-1 -1 -1 -1])*k_m;
% thrust allocation matrix
T = kron([0;0;1],c_direction)*k_t;

A = [T(3,:); M];
Alloc = inv(A);

%% Initial conditions

% initial position [m]
p_0 = [
    0
    0
    0
    ];
% initial quaternion
q_0 = [
    1
    0
    0
    0
    ];
% initial linear velocity (world frame) [m/s]
v_0 = [
    0
    0
    0
    ];
% initial angular velocity (body frame) [rad/s]
omega_0 = [
    0
    0
    0
    ];
% initial tilting angles [rad]
alpha_0 = [
    0
    0
    0
    0
    ];
alpha_0 = alpha_0(1:N);
% initial rate of the tilting angles [rad/s]
alpha_dot_0 = [
    0
    0
    0
    0
    ];
alpha_dot_0 = alpha_dot_0(1:N);
% initial propellers spinning rate [rad/s]
w_bar_0 = [
    w_bar_hover_planar
    w_bar_hover_planar
    -(w_bar_hover_planar)
    -(w_bar_hover_planar)
    ];
w_bar_0 = w_bar_0(1:N);

% initial state vector
x_0 = [
    p_0
    q_0
    v_0
    omega_0
    alpha_0
    alpha_dot_0
    w_bar_0
    ];

%% Initial inputs

% % initial acceleration of the tilting angles [rad/s^2]
% alpha_ddot_0 = [
%     0
%     0
%     0
%     0
%     ];
% 
% % initial acceleration of the propellers [ras/s^2]
% w_bar_dot_0 = [
%     0
%     0
%     0
%     0
%     ];
% 
% % initial input vector
% u_0 = [
%     alpha_ddot_0
%     w_bar_dot_0
%     ];

% initial acceleration of the tilting angles [rad/s^2]
tau_i_x_0 = [
    0
    0
    0
    0
    ];
tau_i_x_0 = tau_i_x_0(1:N);

% initial acceleration of the propellers [ras/s^2]
tau_i_z_0 = 0.45*[
    1
    1
    -1
    -1
    ];
tau_i_z_0 = tau_i_z_0(1:N);

% initial input vector
u_0 = [
    tau_i_x_0
    tau_i_z_0
    ];

T_s = 1e-4;