%% motor constants
wc = 1000;
%controller constants
% Kp_q = .010;
% Kp_d = .010;
Kp_q = 10;
Kp_d = 10;
Ki_q = 0.00001;
Ki_d = 0.00001;

%
Km = .00149;
P = 2;
psi = 0.00152788745;
R = .86*3/2;
L = 0.00001462*3/2;

% friction coefficient

% rated_efficiency = 72.1/100;
% rated_torque = 1.19;   %mNm
% rated_speed = 43450; %rpm

% full_efficiency_torque = (rated_torque/rated_efficiency);
% friction_torque = ((1-rated_efficiency)*full_efficiency_torque);
% F =  (friction_torque/1000)/((rated_speed/60)*2*pi);

% inertia coefficient (RANDOM BALLPARK GUESS)
F = 1e-7;
J = 1e-9;

% L_obs = .00001;
% R_obs = .8;
% psi_obs = .0011;
err_gain = 1;
L_obs = L;
R_obs = R;
psi_obs = .0018;