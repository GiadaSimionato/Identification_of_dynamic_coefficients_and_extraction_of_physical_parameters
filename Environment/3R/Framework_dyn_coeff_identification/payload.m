 @author Giada Simionato, 1822614.
% Script that identifies dynamic coefficients and extracts parameteres for
% a symmetric payload attached to the end-effector of a 3R manipulator.

clear all
close all
clc

CurPath = pwd();
addpath([CurPath, '/../Framework_dyn_model']);

%% Data structures 

pi_r_sym = load('pi_r_sym.mat');
pi_r_sym = pi_r_sym.pi_r_sym;
pi_r_hat = load('pi_r_hat.mat');
pi_r_hat = pi_r_hat.pi_r_hat;
Yr = load('Yr.mat');
Yr = Yr.Yr;

syms m1 m2 m3 mL real                                  % init masses
syms c1x c1y c1z c2x c2y c2z c3x c3y c3z cLx cLy cLz real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz JLxx JLxy JLxz JLyy JLyz JLzz real % init inertia tensors elems wrt RFi (assumption: real)
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real       % joint positions, velocities and accelerations
syms a2 a3 g0 real                                  % DH parameters
syms m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z real
syms mLcLx mLcLy mLcLz real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];

p1 = [m1; m2; m3];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z];
p2_mod = [m1c1x; m1c1y; m1c1z; m2c2x; m2c2y; m2c2z; m3c3x; m3c3y; m3c3z];
p3 = [J1; J2; J3];
pL = [mL; mLcLx; mLcLy; mLcLz; JLxx; JLxy; JLxz; JLyy; JLyz; JLzz];

p = [p1; p2; p3];        % vector of dynamic parameters
p_mod = [p1; p2_mod; p3];

q = [q1, q2, q3];
dq = [dq1, dq2, dq3];
ddq = [ddq1, ddq2, ddq3]; 
dh_sym = [a2, a3];      % vector of kinematic parameters

pi_r_sym = subs(pi_r_sym, p2_mod, [m1*c1x; m1*c1y; m1*c1z; m2*c2x; m2*c2y; m2*c2z; m3*c3x; m3*c3y; m3*c3z]);
pi_r_sym_L = subs(pi_r_sym, [m3, c3x, c3y, c3z, J3xx, J3xy, J3xz, J3yy, J3yz, J3zz], [m3+mL, (c3x*m3 + cLx*mL)/(m3+mL), (c3y*m3 + cLy*mL)/(m3+mL), (c3z*m3 + cLz*mL)/(m3+mL), J3xx+JLxx, J3xy+JLxy, J3xz+JLxz, J3yy+JLyy, J3yz+JLyz, J3zz+JLzz]);
eps_sym = pi_r_sym - pi_r_sym_L;
nz_rows = not(eps_sym(:, 1)==0);
eps_nz = eps_sym(nz_rows);
eps_nz = subs(eps_nz, [mL*cLx mL*cLy mL*cLz], [mLcLx mLcLy mLcLz]);
J_eps = jacobian(eps_nz, pL);

%% Numerical values

masses_values = [3.5, 0.8, 1.2];  % values for m1, m2, m3 [kg]
com_values = [1e-03, -12e-02, 2e-03, -88e-03, 1e-03, 2e-03, -6e-02, 2e-03, 3e-03];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [5.64e-02, 3.15e-04, -1.4e-05, 2.63e-02, 6.3e-04, 7.98e-02, 4.98e-04, 1.27e-04, 2.54e-04, 1.34e-02, -3.2e-06, 1.34e-02, 5.71e-04, 2.52e-04, 3.78e-04, 9.02e-03, -1.44e-05, 9.01e-03]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.22, 0.15]; % values for a2, a3 [m]
ctm_values = [3.5*1e-03, -12e-02*3.5, 2e-03*3.5, -88e-03*0.8, 1e-03*0.8, 2e-03*0.8, -6e-02*1.2, 2e-03*1.2, 3e-03*1.2];
mL_val = 1.5; %[kg]
com_L_vals = [0.003, 0, 0]; % [m] simmetric payload
inertia_L_vals = [0.0001813, 0, 0, 0.0001785, 0, 0.0001806];

masses_final = [3.5, 0.8, 1.2+mL_val];
com_final = [1e-03, -12e-02, 2e-03, -88e-03, 1e-03, 2e-03, (-6e-02*1.2 + 0.003*mL_val)/(1.2+mL_val), (2e-03*1.2)/(1.2+mL_val), (3e-03*1.2)/(1.2+mL_val)];
inertia_final = [5.64e-02, 3.15e-04, -1.4e-05, 2.63e-02, 6.3e-04, 7.98e-02, 4.98e-04, 1.27e-04, 2.54e-04, 1.34e-02, -3.2e-06, 1.34e-02, 5.71e-04+ 0.0001813, 2.52e-04, 3.78e-04, 9.02e-03+ 0.0001785, -1.44e-05, 9.01e-03+0.0001806];
%% Identification

tau = load('dyn_mod_3R.mat', 'tau');
tau = expand(tau.tau);

%% Design trajectories

disp('Designing trajectories...');
n_traj = 2;              % number of trajectories to be generated for identification
[q_k,dq_k,ddq_k] = generate_exc_traj(n_traj);  % 
save('q_traj_PAYLOAD.mat', 'q_k');
save('dq_traj_PAYLOAD.mat', 'dq_k');
save('ddq_traj_PAYLOAD.mat', 'ddq_k');

%% Get stacked regressor and torque

Y_val = subs(Yr, cat(2, dh_sym, g0), cat(2, dh_values, 9.81));  % subs values of DH params and gravity in Y 
tau_val = subs(tau, cat(2, p.', dh_sym, g0), cat(2, masses_final, com_final, inertia_final, dh_values, 9.81)); % subs all parameters values (included DH and g0) 
params_joint_sym = cat(2, q, dq, ddq);

Yr_stacked = get_stacked_numerical(Y_val, q_k, dq_k, ddq_k, params_joint_sym);
tau_stacked = get_stacked_numerical(tau_val, q_k, dq_k, ddq_k, params_joint_sym);

%% 
pi_L_hat = pinv(Yr_stacked)*tau_stacked;
save('pi_L_hat.mat', 'pi_L_hat');
% eps_hat = pi_L_hat - pi_r_hat;
% eps_hat = eps_hat(nz_rows);
% p_L_hat = pinv(J_eps)*eps_hat;

%%
p_L_hat(2) = p_L_hat(2)/p_L_hat(1);
p_L_hat(3) = p_L_hat(3)/p_L_hat(1);
p_L_hat(4) = p_L_hat(4)/p_L_hat(1);
%%
error = double(cat(2, mL_val, com_L_vals, inertia_L_vals).'-p_L_hat);

%%
% pi_L_recons = subs(pi_r_sym_L, p.', cat(2, masses_values, com_values, inertia_values));
% pi_L_recons = subs(pi_L_recons,[mL, cLx, cLy, cLz, JLxx, JLxy, JLxz, JLyy, JLyz, JLzz] ,cat(2, mL_val, com_L_vals, inertia_L_vals));
% pi_L_recons = double(pi_L_recons);
% error_pi = pi_L_recons - pi_L_hat;  % error identification coeff
% eps_sym = subs(eps_sym, [mL*cLx mL*cLy mL*cLz], [mLcLx mLcLy mLcLz]);
% J_eps = jacobian(eps_sym, pL);
% eps_hat = pi_L_hat - pi_r_hat;
% p_L_hat = pinv(J_eps)*eps_hat;


