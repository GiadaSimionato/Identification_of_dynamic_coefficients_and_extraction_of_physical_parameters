clear all
close all
clc


syms q1 q2 q3 q4 q5 q6 q7 dq1 dq2 dq3 dq4 dq5 dq6 dq7 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 real       % joint positions, velocities and accelerations
syms d1 d2 g0 real                                  % DH parameters
dh_sym = [d1, d2];      % vector of kinematic parameters
dh_values = [0.4, 0.39]; % values for d1, d2 [m]

q = [q1, q2, q3, q4, q5, q6, q7];
dq = [dq1, dq2, dq3, dq4, dq5, dq6, dq7];
ddq = [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ddq7]; 

Q = load('q_traj_valid.mat');
Q = Q.Q;
dQ = load('dq_traj_valid.mat');
dQ = dQ.dQ;
ddQ = load('ddq_traj_valid.mat');
ddQ = ddQ.ddQ;

pi_r_sym = load('pi_r_sym.mat');
pi_r_sym = pi_r_sym.pi_r_sym;

pi_r_hat = load('pi_r_hat.mat');
pi_r_hat = pi_r_hat.pi_r_hat;

Yr = load('Yr.mat');
Yr = Yr.Yr;
Yr = subs(Yr, cat(2, dh_sym, g0), cat(2, dh_values, 9.81));

joint_symbols = cat(2, q, dq, ddq);

%%
n_joints = 7;

syms m1 m2 m3 m4 m5 m6 m7 real                                  % init masses
syms c1x c1y c1z c2x c2y c2z c3x c3y c3z c4x c4y c4z c5x c5y c5z c6x c6y c6z c7x c7y c7z real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz real % init inertia tensors elems wrt RFi (assumption: real)
syms J4xx J4xy J4xz J4yy J4yz J4zz J5xx J5xy J5xz J5yy J5yz J5zz J6xx J6xy J6xz J6yy J6yz J6zz J7xx J7xy J7xz J7yy J7yz J7zz real
syms q1 q2 q3 q4 q5 q6 q7 dq1 dq2 dq3 dq4 dq5 dq6 dq7 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 real       % joint positions, velocities and accelerations
syms d1 d2 g0 real                                  % DH parameters
syms m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z real
syms m4c4x m4c4y m4c4z m5c5x m5c5y m5c5z m6c6x m6c6y m6c6z m7c7x m7c7y m7c7z real
syms fv1 fv2 fv3 fv4 fv5 fv6 fv7 fc1 fc2 fc3 fc4 fc5 fc6 fc7 fo1 fo2 fo3 fo4 fo5 fo6 fo7 real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];
J4 = [J4xx; J4xy; J4xz; J4yy; J4yz; J4zz];
J5 = [J5xx; J5xy; J5xz; J5yy; J5yz; J5zz];
J6 = [J6xx; J6xy; J6xz; J6yy; J6yz; J6zz];
J7 = [J7xx; J7xy; J7xz; J7yy; J7yz; J7zz];

p1 = [m1; m2; m3; m4; m5; m6; m7];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z; c4x; c4y; c4z; c5x; c5y; c5z; c6x; c6y; c6z; c7x; c7y; c7z];
p2_mod = [m1c1x; m1c1y; m1c1z; m2c2x; m2c2y; m2c2z; m3c3x; m3c3y; m3c3z; m4c4x; m4c4y; m4c4z; m5c5x; m5c5y; m5c5z; m6c6x; m6c6y; m6c6z; m7c7x; m7c7y; m7c7z;];
p3 = [J1; J2; J3; J4; J5; J6; J7];
p4 = [fv1; fv2; fv3; fv4; fv5; fv6; fv7];
p5 = [fc1; fc2; fc3; fc4; fc5; fc6; fc7];
p6 = [fo1; fo2; fo3; fo4; fo5; fo6; fo7];

p_mod = [p1; p2_mod; p3; p4; p5; p6];        % vector of dynamic parameters

masses_values = [2.7, 2.7, 2.7, 2.7, 1.7, 1.6, 0.3];  % values for m1, m2, m3 [kg]
com_values = [0.001340, -0.087777, -0.026220, 0.001340, -0.026220, 0.087777, -0.00134, 0.087777, -0.026220, -0.001340, 0.026220, 0.087777, -0.000993, -0.11165, -0.026958, -0.000259, -0.005956, -0.005328, 0, 0, 0.063];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [0.039, 3.206e-04, 9.415e-05, 6.887e-03, -2.681e-03, 0.03698, 0.039, 9.415e-05, -3.145e-04, 0.03698, 9.747e-03, 6.887e-03, 6.887e-03, 3.145e-04, -9.5572e-05, 6.8872e-03, 2.681e-03, 0.037, 0.037, 9.415e-05, 3.1455e-04, 0.037, 9.747e-03, 6.887e-03, 0.032, -1.898e-04, -4.474e-05, 4.945e-03, -2.023e-03, 0.03, 0.003, -2.463e-06, -2.323e-06, 3.068e-03, -7.008e-05, 3.47e-03, 3.47e-03, 0, 0, 1.292e-03, 0, 1.584e-04]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.4, 0.39]; % values for d1, d2 [m]
ctm_values = [0.001340*2.7, -0.087777*2.7, -0.026220*2.7, 0.001340*2.7, -0.026220*2.7, 0.087777*2.7, -0.00134*2.7, 0.087777*2.7, -0.026220*2.7, -0.001340*2.7, 0.026220*2.7, 0.087777*2.7, -0.000993*1.7, -0.11165*1.7, -0.026958*1.7, -0.000259*1.6, -0.005956*1.6, -0.005328*1.6, 0, 0, 0.063*0.3];
fv_values = [0.0665, 0.1987, 0.0399, 0.2257, 0.1023, -0.0132, 0.0638];
fc_values = [0.245, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109];
fo_values = [-0.1073, -0.1566, -0.0686, -0.2522, 0.0045, 0.0910, -0.0127];

pi_r_recon = double(subs(pi_r_sym, p_mod.', cat(2, masses_values, ctm_values, inertia_values, fv_values, fc_values, fo_values)));

%%
% [tau_est1, tau_real1, timesteps_1] = valid1(pi_r_hat, pi_r_recon, Yr, joint_symbols, Q(:, 1:100), dQ(:, 1:100), ddQ(:, 1:100));
% save('tau_est1.mat', 'tau_est1');
% save('tau_real1.mat', 'tau_real1');
[tau_est2, tau_real2, timesteps_2] = valid1(pi_r_hat, pi_r_recon, Yr, joint_symbols, Q(:, 101:200), dQ(:, 101:200), ddQ(:, 101:200));
save('tau_est2.mat', 'tau_est2');
save('tau_real2.mat', 'tau_real2');
% [tau_est3, tau_real3, timesteps_3] = valid1(pi_r_hat, pi_r_recon, Yr, joint_symbols, Q(:, 1:100), dQ(:, 201:300), ddQ(:, 201:300));
% save('tau_est3.mat', 'tau_est3');
% save('tau_real3.mat', 'tau_real3');
% [tau_est4, tau_real4, timesteps_4] = valid1(pi_r_hat, pi_r_recon, Yr, joint_symbols, Q(:, 1:100), dQ(:, 301:401), ddQ(:, 301:401));
% save('tau_est4.mat', 'tau_est4');
% save('tau_real4.mat', 'tau_real4');
