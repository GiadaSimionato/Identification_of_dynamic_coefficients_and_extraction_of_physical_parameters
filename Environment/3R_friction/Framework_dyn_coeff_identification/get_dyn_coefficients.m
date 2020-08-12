% @author Giada Simionato, 1822614.
% Script that identifies the dynamic coefficients for a friction 3R
% spatial anthropomorphic manipulator.

clear all
close all
clc

CurPath = pwd();
addpath([CurPath, '/../Framework_dyn_model']);

format long g

%% Initialize dynamic parameters and data structures

n_joints = 3;

syms m1 m2 m3 real                                  % init masses
syms c1x c1y c1z c2x c2y c2z c3x c3y c3z real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz real % init inertia tensors elems wrt RFi (assumption: real)
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real       % joint positions, velocities and accelerations
syms a2 a3 g0 real                                  % DH parameters
syms fv1 fv2 fv3 real     % viscous friction coefficients
syms fc1 fc2 fc3 real     % Coulomb friction coefficients
syms fo1 fo2 fo3 real     % Coulomb friction offsets
syms m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];

p1 = [m1; m2; m3];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z];
p2_mod = [m1c1x; m1c1y; m1c1z; m2c2x; m2c2y; m2c2z; m3c3x; m3c3y; m3c3z];
p3 = [J1; J2; J3];
p4 = [fv1; fv2; fv3];     % vector of viscous friction parameters
p5 = [fc1; fc2; fc3];     % vector of Coulomb friction parameters
p6 = [fo1; fo2; fo3];     % vector of offsets for Coulomb friction

p = [p1; p2; p3; p4; p5; p6];        % vector of dynamic parameters
p_mod = [p1; p2_mod; p3; p4; p5; p6];

q = [q1, q2, q3];
dq = [dq1, dq2, dq3];
ddq = [ddq1, ddq2, ddq3]; 
dh_sym = [a2, a3];      % vector of kinematic parameters

%% Define real values for dynamic parameters

masses_values = [3.5, 0.8, 1.2];  % values for m1, m2, m3 [kg]
com_values = [1e-03, -12e-02, 2e-03, -88e-03, 1e-03, 2e-03, -6e-02, 2e-03, 3e-03];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [5.64e-02, 3.15e-04, -1.4e-05, 2.63e-02, 6.3e-04, 7.98e-02, 4.98e-04, 1.27e-04, 2.54e-04, 1.34e-02, -3.2e-06, 1.34e-02, 5.71e-04, 2.52e-04, 3.78e-04, 9.02e-03, -1.44e-05, 9.01e-03]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.22, 0.15]; % values for a2, a3 [m]
ctm_values = [3.5*1e-03, -12e-02*3.5, 2e-03*3.5, -88e-03*0.8, 1e-03*0.8, 2e-03*0.8, -6e-02*1.2, 2e-03*1.2, 3e-03*1.2];
fv_values = [0.0665, 0.1987, 0.0399];
fc_values = [0.245, 0.1523, 0.1827];
fo_values = [-0.1073, -0.1566, -0.0686];

%% Get symbolic dynamic model

% tau = expand(NE());
tau = load('dyn_mod_3R_friction.mat', 'tau');
tau = expand(tau.tau);
tau_mod = subs(tau, [m1*c1x; m1*c1y; m1*c1z; m2*c2x; m2*c2y; m2*c2z; m3*c3x; m3*c3y; m3*c3z], p2_mod);

%% Get symbolic regressor

Y = jacobian(tau_mod, p_mod);

%% Design trajectories

disp('Designing trajectories...');
n_traj = 3;              % number of trajectories to be generated for identification
[q_k,dq_k,ddq_k] = generate_exc_traj(n_traj);  % get sampled trajectories (2001 samples per traj per joint, 3 trajs, 3 joints
save('q_traj.mat', 'q_k');
save('dq_traj.mat', 'dq_k');
save('ddq_traj.mat', 'ddq_k');
% q_k = load('q_traj.mat');
% q_k = q_k.q_k;
% dq_k = load('dq_traj.mat');
% dq_k = dq_k.dq_k;
% ddq_k = load('ddq_traj.mat');
% ddq_k = ddq_k.ddq_k;

%% Get stacked regressor and torque

Y_val = subs(Y, cat(2, dh_sym, g0), cat(2, dh_values, 9.81));  % subs values of DH params and gravity in Y 
tau_val = subs(tau, cat(2, p.', dh_sym, g0), cat(2, masses_values, com_values, inertia_values, fv_values, fc_values, fo_values, dh_values, 9.81)); % subs all parameters values (included DH and g0) 
params_joint_sym = cat(2, q, dq, ddq);

Y_stacked = get_stacked_numerical(Y_val, q_k, dq_k, ddq_k, params_joint_sym);
tau_stacked = get_stacked_numerical(tau_val, q_k, dq_k, ddq_k, params_joint_sym);


%% Get full column rank numerical regressor and dynamic coefficients estimation

[Y_rref, li_cols] = rref(Y_stacked);          % Gauss-Jordan elimination       
Yr_stacked = Y_stacked(:, li_cols);           % reduced numeric stacked regressor
Yr_stacked_pinv = pinv(Yr_stacked);           % pseudoinverted numerical reduced stacked regressor
pi_r_hat = Yr_stacked_pinv*tau_stacked;       % dynamic coefficients estimate (numerical)  (HERE FOR NUMERICAL VALUE COEFF.)
save('pi_r_hat.mat', 'pi_r_hat');

%% Get symbolic form of dynamic coefficients

Y_rref(abs(Y_rref)<1e-10) = 0;       % set to zero coefficients smaller than threshold
Yr = Y(:, li_cols);                  % reduced symbolic regressor (only linearly independent columns)
save('Yr.mat', 'Yr'); 
pi_r = Y_rref*p_mod;   
pi_r_sym = pi_r(1:length(li_cols));  % symbolic form of the dynamic coefficients (HERE FOR SYMBOLIC FORM COEFF)
save('pi_r_sym.mat', 'pi_r_sym');

%% Error computation
pi_r_sym = load('pi_r_sym.mat');
pi_r_sym = pi_r_sym.pi_r_sym;
pi_r_hat = load('pi_r_hat.mat');
pi_r_hat = pi_r_hat.pi_r_hat;
pi_r_reconstructed = double(subs(pi_r_sym, p_mod.', cat(2, masses_values, ctm_values, inertia_values, fv_values, fc_values, fo_values)));
error = pi_r_reconstructed-pi_r_hat;

%% Dynamic coefficients validation

[tau_est, tau_real, tau_tot, timesteps] = coeff_validation(pi_r_hat, pi_r_reconstructed, Yr, tau, cat(2, params_joint_sym, p.', dh_sym, g0) , cat(2, masses_values, com_values, inertia_values, fv_values, fc_values, fo_values, dh_values, 9.81));

%% Plot torques estimated and obtained from dynamic model

duration = 10;
margin = 0.005;
figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Torques comparison', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(3,1,j)
	plot(timesteps, tau_est(j:3:length(tau_est)), 'r', timesteps, tau_real(j:3:length(tau_real)), 'b:', timesteps, tau_tot(j:3:length(tau_tot)), 'g--');
	grid on
    xlabel('t [s]')
	ylabel(strcat('tau_{',num2str(j),'} [Nm]'))
    xlim([-margin, duration+margin])
    ylim([min(min(tau_est(j:3:length(tau_est))), min(tau_real(j:3:length(tau_real))))-margin, max(max(tau_est(j:3:length(tau_est))), max(tau_real(j:3:length(tau_real))))+margin])
end
legend('Estimated', 'Reconstructed', 'Real');