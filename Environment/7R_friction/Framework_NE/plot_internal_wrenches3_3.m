% @author Giada Simionato, 1822614.
% Script that plots internal forces in 3R manipulator

clear all
close all
clc

CurPath = pwd();
addpath([CurPath, '/../Framework_dyn_params_extraction']);
addpath([CurPath, '/../Data']);

%% Symbolic dynamic parameters initialization

syms g0 q1 q2 q3 q4 q5 q6 q7 dq1 dq2 dq3 dq4 dq5 dq6 dq7 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 ...
    m1 m2 m3 m4 m5 m6 m7 d1 d2 ...
    J1xx J1xy J1xz J1yy J1yz J1zz ...
    J2xx J2xy J2xz J2yy J2yz J2zz ...
    J3xx J3xy J3xz J3yy J3yz J3zz ...
    J4xx J4xy J4xz J4yy J4yz J4zz ...
    J5xx J5xy J5xz J5yy J5yz J5zz ...
    J6xx J6xy J6xz J6yy J6yz J6zz ...
    J7xx J7xy J7xz J7yy J7yz J7zz ...
    c1x c1y c1z c2x c2y c2z c3x c3y c3z ...
    c4x c4y c4z c5x c5y c5z c6x c6y c6z c7x c7y c7z ...
    fv1 fv2 fv3 fv4 fv5 fv6 fv7 ...
    fc1 fc2 fc3 fc4 fc5 fc6 fc7 ...
    fo1 fo2 fo3 fo4 fo5 fo6 fo7 real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];
J4 = [J4xx; J4xy; J4xz; J4yy; J4yz; J4zz];
J5 = [J5xx; J5xy; J5xz; J5yy; J5yz; J5zz];
J6 = [J6xx; J6xy; J6xz; J6yy; J6yz; J6zz];
J7 = [J7xx; J7xy; J7xz; J7yy; J7yz; J7zz];

p1 = [m1; m2; m3; m4; m5; m6; m7];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z; c4x; c4y; c4z; c5x; c5y; c5z; c6x; c6y; c6z; c7x; c7y; c7z];
p3 = [J1; J2; J3; J4; J5; J6; J7];
p4 = [fv1; fv2; fv3; fv4; fv5; fv6; fv7];
p5 = [fc1; fc2; fc3; fc4; fc5; fc6; fc7];
p6 = [fo1; fo2; fo3; fo4; fo5; fo6; fo7];

p = [p1; p2; p3; p4; p5; p6];        % vector of dynamic parameters

p = p.';

q = [q1, q2, q3, q4, q5, q6, q7];
dq = [dq1, dq2, dq3, dq4, dq5, dq6, dq7];
ddq = [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ddq7]; 
dh_sym = [d1, d2];      % vector of kinematic parameters

joint_params_sym = cat(2, q, dq, ddq);
fixed_params = cat(2, dh_sym, g0);

%% Real dynamic parameters

masses_values = [2.7, 2.7, 2.7, 2.7, 1.7, 1.6, 0.3];  % values for m1, m2, m3 [kg]
com_values = [0.001340, -0.087777, -0.026220, 0.001340, -0.026220, 0.087777, -0.00134, 0.087777, -0.026220, -0.001340, 0.026220, 0.087777, -0.000993, -0.11165, -0.026958, -0.000259, -0.005956, -0.005328, 0, 0, 0.063];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [0.039, 3.206e-04, 9.415e-05, 6.887e-03, -2.681e-03, 0.03698, 0.039, 9.415e-05, -3.145e-04, 0.03698, 9.747e-03, 6.887e-03, 6.887e-03, 3.145e-04, -9.5572e-05, 6.8872e-03, 2.681e-03, 0.037, 0.037, 9.415e-05, 3.1455e-04, 0.037, 9.747e-03, 6.887e-03, 0.032, -1.898e-04, -4.474e-05, 4.945e-03, -2.023e-03, 0.03, 0.003, -2.463e-06, -2.323e-06, 3.068e-03, -7.008e-05, 3.47e-03, 3.47e-03, 0, 0, 1.292e-03, 0, 1.584e-04]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.4, 0.39]; % values for d1, d2 [m]
fv_values = [0.0665, 0.1987, 0.0399, 0.2257, 0.1023, -0.0132, 0.0638];
fc_values = [0.245, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109];
fo_values = [-0.1073, -0.1566, -0.0686, -0.2522, 0.0045, 0.0910, -0.0127];

p_values_real = cat(2, masses_values, com_values, inertia_values, fv_values, fc_values, fo_values);
fixed_values = cat(2, dh_values, 9.81);

%% Extracted dynamic parameters

% p_values_feasible = load('../Data/results_sim/normal/3R_4_params.mat');
% p_values_feasible = p_values_feasible.dyn_parameters.';
% p_values_unfeasible = load('../Data/results_sim/loose/3R_9_params.mat');
% p_values_unfeasible = p_values_unfeasible.dyn_parameters.';
p_25 = load('../Data/results_sim/normal/7R_37_params.mat');
p_25 = p_25.dyn_parameters.';
p_28 = load('../Data/results_sim/normal/7R_40_params.mat');
p_28 = p_28.dyn_parameters.';
p_29 = load('../Data/results_sim/tight/7R_41_params.mat');
p_29 = p_29.dyn_parameters.';
p_32 = load('../Data/results_sim/tight/7R_44_params.mat');
p_32 = p_32.dyn_parameters.';
p_33 = load('../Data/results_sim/loose/7R_45_params.mat');
p_33 = p_33.dyn_parameters.';
p_36 = load('../Data/results_sim/loose/7R_48_params.mat');
p_36 = p_36.dyn_parameters.';

%% NE algorithm

[tau, forces_momenta] = NE();
% tau = expand(tau);
% forces_momenta = expand(forces_momenta);

%% Customize forces_momenta

fm_fixed_value = subs(forces_momenta, fixed_params, fixed_values);

% fm_real_value = subs(fm_fixed_value, p, p_values_real);
% fm_feasible_value = subs(fm_fixed_value, p, p_values_feasible);
% fm_unfeasible_value = subs(fm_fixed_value, p, p_values_unfeasible);

fm_real_value = subs(fm_fixed_value, p, p_values_real);

fm_25 = subs(fm_fixed_value, p, p_25);
fm_28 = subs(fm_fixed_value, p, p_28);
fm_29 = subs(fm_fixed_value, p, p_29);
fm_32 = subs(fm_fixed_value, p, p_32);
fm_33 = subs(fm_fixed_value, p, p_33);
fm_36 = subs(fm_fixed_value, p, p_36);

%% Get sinusoidal trajectory

% [qk, dqk, ddqk, time_instants] = get_sinusoidal_traj();


qk = load('../Data/traj_final_NE/q.mat');
qk = qk.q;
dqk = load('../Data/traj_final_NE/dq.mat');
dqk = dqk.dq;
ddqk = load('../Data/traj_final_NE/ddq.mat');
ddqk = ddqk.ddq;
time_instants = load('../Data/traj_final_NE/time_instants.mat');
time_instants = time_instants.time_instants;



%%

%[fm_real, fm_feasible, fm_unfeasible] = get_behaviour(fm_real_value, fm_feasible_value, fm_unfeasible_value, joint_params_sym, qk, dqk, ddqk, time_instants);
[fm_real_3, fm_25_3] = get_behaviour(fm_real_value, fm_25, joint_params_sym, qk(:, 101:150), dqk(:, 101:150), ddqk(:, 101:150), time_instants(:, 101:150));
save('fm_real_3.mat', 'fm_real_3');
save('fm_25_3.mat', 'fm_25_3');