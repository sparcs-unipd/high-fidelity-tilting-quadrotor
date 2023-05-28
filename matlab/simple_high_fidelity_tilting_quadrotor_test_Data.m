% motor+propeller inertia matrix [kg*m^2]
%I_i = diag([3.8464910483993325e-07 2.649858234714004e-05 2.649858234714004e-05]);
%I_i = diag([2.649858234714004e-05 2.649858234714004e-05 2.649858234714004e-06]);

skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

N = 4;

% propeller data
prop.m = 5e-3;      % propeller mass [kg]
prop.R = 80e-3;     % propeller Radius [m]
prop.h = 10e-3;     % propeller height [m]
I_prop = diag([ ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/2*prop.m*prop.R^2]);
rotor.m = 10e-3;    % rotor mass [kg]
rotor.R = 20e-3;    % rotor Radius [m]
rotor.h = 10e-3;    % rotor height [m]
I_rotor = diag([ ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/2*rotor.m*rotor.R^2]);
I_i = I_prop+I_rotor;
stator.m = 50e-3;   % stator mass [kg]

prop.w_max_rpm = 10000;                 % propeller maximum rate [rpm]
prop.w_max = prop.w_max_rpm * 2*pi/60;  % propeller maximum rate [rad/s]

prop.max_thrust = 6*9.81/4;             % propeller maximum thrust [N]           

k_t = prop.max_thrust / prop.w_max.^2;  % thrust coefficient [N/s^2]
k_m = k_t * 0.06;                       % propeller drag coefficient [Nm/s^2]

% propeller spinning convention
% 1 if positive velocity (CCW) gives positive thrust
% -1 if negative velocity (CW) gives positive thrust
c_direction = [1 1 -1 -1];


% main body data
%I_b = diag([0.029125 0.029125 0.055225]);
%I_b = diag([0.029125 0.029125 0.029125]);
body.m = 2;             % body mass [kg]
body.R = 100e-3;        % body Radius [m]
body.h = 50e-3;         % body height [m]
I_b = diag([ ...        % body inertia [kg*m^2]
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/2*body.m*body.R^2]);
m_b = body.m + N*(stator.m+rotor.m+prop.m);       % body mass


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

for k=1:4
    I_b = I_b - (stator.m+rotor.m+prop.m)*skew(p_mot(k,:))^2;
end

% HOVER DATA
required_hover_thrust = m_b * 9.81;
w_bar_hover_planar = sqrt(required_hover_thrust/4/k_t);

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
alpha_0 = 0*[
    0.1
    0.1
    0.1
    0.1
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
    -w_bar_hover_planar
    -w_bar_hover_planar
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