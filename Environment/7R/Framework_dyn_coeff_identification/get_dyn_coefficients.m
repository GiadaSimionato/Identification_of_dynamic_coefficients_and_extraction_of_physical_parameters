% @author Giada Simionato, 1822614.
% Script that identifies the dynamic coefficients for a frictionless 7R manipulator.

clear all
close all
clc

CurPath = pwd();
addpath([CurPath, '/../Framework_dyn_model']);

format long g

%% Initialize dynamic parameters and data structures
disp('1');
n_joints = 7;

syms m1 m2 m3 m4 m5 m6 m7 real                                  % init masses
syms c1x c1y c1z c2x c2y c2z c3x c3y c3z c4x c4y c4z c5x c5y c5z c6x c6y c6z c7x c7y c7z real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz real % init inertia tensors elems wrt RFi (assumption: real)
syms J4xx J4xy J4xz J4yy J4yz J4zz J5xx J5xy J5xz J5yy J5yz J5zz J6xx J6xy J6xz J6yy J6yz J6zz J7xx J7xy J7xz J7yy J7yz J7zz real
syms q1 q2 q3 q4 q5 q6 q7 dq1 dq2 dq3 dq4 dq5 dq6 dq7 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 real       % joint positions, velocities and accelerations
syms d1 d2 g0 real                                  % DH parameters
syms m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z real
syms m4c4x m4c4y m4c4z m5c5x m5c5y m5c5z m6c6x m6c6y m6c6z m7c7x m7c7y m7c7z real

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

p = [p1; p2; p3];        % vector of dynamic parameters
p_mod = [p1; p2_mod; p3];

q = [q1, q2, q3, q4, q5, q6, q7];
dq = [dq1, dq2, dq3, dq4, dq5, dq6, dq7];
ddq = [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ddq7]; 
dh_sym = [d1, d2];      % vector of kinematic parameters

%% Define real values for dynamic parameters
disp('2');
masses_values = [2.7, 2.7, 2.7, 2.7, 1.7, 1.6, 0.3];  % values for m1, m2, m3 [kg]
com_values = [0.001340, -0.087777, -0.026220, 0.001340, -0.026220, 0.087777, -0.00134, 0.087777, -0.026220, -0.001340, 0.026220, 0.087777, -0.000993, -0.11165, -0.026958, -0.000259, -0.005956, -0.005328, 0, 0, 0.063];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [0.039, 3.206e-04, 9.415e-05, 6.887e-03, -2.681e-03, 0.03698, 0.039, 9.415e-05, -3.145e-04, 0.03698, 9.747e-03, 6.887e-03, 6.887e-03, 3.145e-04, -9.5572e-05, 6.8872e-03, 2.681e-03, 0.037, 0.037, 9.415e-05, 3.1455e-04, 0.037, 9.747e-03, 6.887e-03, 0.032, -1.898e-04, -4.474e-05, 4.945e-03, -2.023e-03, 0.03, 0.003, -2.463e-06, -2.323e-06, 3.068e-03, -7.008e-05, 3.47e-03, 3.47e-03, 0, 0, 1.292e-03, 0, 1.584e-04]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.4, 0.39]; % values for d1, d2 [m]
ctm_values = [0.001340*2.7, -0.087777*2.7, -0.026220*2.7, 0.001340*2.7, -0.026220*2.7, 0.087777*2.7, -0.00134*2.7, 0.087777*2.7, -0.026220*2.7, -0.001340*2.7, 0.026220*2.7, 0.087777*2.7, -0.000993*1.7, -0.11165*1.7, -0.026958*1.7, -0.000259*1.6, -0.005956*1.6, -0.005328*1.6, 0, 0, 0.063*0.3];

%% Get symbolic dynamic model
disp('3');
% tau = expand(NE());
tau = load('dyn_model_7R.mat', 'tau');
tau = expand(tau.tau);
tau_mod = subs(tau, [m1*c1x; m1*c1y; m1*c1z; m2*c2x; m2*c2y; m2*c2z; m3*c3x; m3*c3y; m3*c3z; m4*c4x; m4*c4y; m4*c4z; m5*c5x; m5*c5y; m5*c5z; m6*c6x; m6*c6y; m6*c6z; m7*c7x; m7*c7y; m7*c7z;], p2_mod);

%% Get symbolic regressor
disp('4');
Y = jacobian(tau_mod, p_mod);

%% Design trajectories
disp('5');
disp('Designing trajectories...');
n_traj = 3;              % number of trajectories to be generated for identification
% [q_k,dq_k,ddq_k] = generate_exc_traj(n_traj);  % get sampled trajectories (2001 samples per traj per joint, 3 trajs, 3 joints
% save('q_traj.mat', 'q_k');
% save('dq_traj.mat', 'dq_k');
% save('ddq_traj.mat', 'ddq_k');
q_k = load('q_traj.mat');
q_k = q_k.q_k;
dq_k = load('dq_traj.mat');
dq_k = dq_k.dq_k;
ddq_k = load('ddq_traj.mat');
ddq_k = ddq_k.ddq_k;

%% Get stacked regressor and torque
disp('6');
Y_val = subs(Y, cat(2, dh_sym, g0), cat(2, dh_values, 9.81));  % subs values of DH params and gravity in Y 
tau_val = subs(tau, cat(2, p.', dh_sym, g0), cat(2, masses_values, com_values, inertia_values, dh_values, 9.81)); % subs all parameters values (included DH and g0) 
params_joint_sym = cat(2, q, dq, ddq);

% [Y_stacked_11, tau_stacked_11] = get_stacked_numerical(Y_val, tau_val, q_k(:,1:147,1), dq_k(:,1:147,1), ddq_k(:,1:147,1), params_joint_sym);
% save('Y11_stacked.mat', 'Y_stacked_11');
% save('tau11_stacked.mat', 'tau_stacked_11');
% [Y_stacked_12, tau_stacked_12] = get_stacked_numerical(Y_val, tau_val, q_k(:,147:295,1), dq_k(:,147:295,1), ddq_k(:,147:295,1), params_joint_sym);
% save('Y12_stacked.mat', 'Y_stacked_12');
% save('tau12_stacked.mat', 'tau_stacked_12');
% [Y_stacked_13, tau_stacked_13] = get_stacked_numerical(Y_val, tau_val, q_k(:,296:442,1), dq_k(:,296:442,1), ddq_k(:,296:442,1), params_joint_sym);
% save('Y13_stacked.mat', 'Y_stacked_13');
% save('tau13_stacked.mat', 'tau_stacked_13');
% [Y_stacked_14, tau_stacked_14] = get_stacked_numerical(Y_val, tau_val, q_k(:,443:589,1), dq_k(:,443:589,1), ddq_k(:,443:589,1), params_joint_sym);
% save('Y14_stacked.mat', 'Y_stacked_14');
% save('tau14_stacked.mat', 'tau_stacked_14');



%[Y_stacked_1, tau_stacked_1] = get_stacked_numerical(Y_val, tau_val, q_k(:,:,1), dq_k(:,:,1), ddq_k(:,:,1), params_joint_sym);
% [Y_stacked_2, tau_stacked_2] = get_stacked_numerical(Y_val, tau_val, q_k(:,:,2), dq_k(:,:,2), ddq_k(:,:,2), params_joint_sym);
% [Y_stacked_3, tau_stacked_3] = get_stacked_numerical(Y_val, tau_val, q_k(:,:,3), dq_k(:,:,3), ddq_k(:,:,3), params_joint_sym);
% save('Y1_stacked.mat', 'Y_stacked_1');
% save('tau1_stacked.mat', 'tau_stacked_1');
% save('Y2_stacked.mat', 'Y_stacked_2');
% save('tau2_stacked.mat', 'tau_stacked_2');
% save('Y3_stacked.mat', 'Y_stacked_3');
% save('tau3_stacked.mat', 'tau_stacked_3');

Y_stacked_11 = load('Y11_stacked.mat');
Y_stacked_11 = Y_stacked_11.Y_stacked_11;
Y_stacked_12 = load('Y12_stacked.mat');
Y_stacked_12 = Y_stacked_12.Y_stacked_12;
Y_stacked_13 = load('Y13_stacked.mat');
Y_stacked_13 = Y_stacked_13.Y_stacked_13;
Y_stacked_14 = load('Y14_stacked.mat');
Y_stacked_14 = Y_stacked_14.Y_stacked_14;
Y_stacked_21 = load('Y21_stacked.mat');
Y_stacked_21 = Y_stacked_21.Y_stacked_21;
Y_stacked_22 = load('Y22_stacked.mat');
Y_stacked_22 = Y_stacked_22.Y_stacked_22;
Y_stacked_23 = load('Y23_stacked.mat');
Y_stacked_23 = Y_stacked_23.Y_stacked_23;
Y_stacked_24 = load('Y24_stacked.mat');
Y_stacked_24 = Y_stacked_24.Y_stacked_24;

tau_stacked_11 = load('tau11_stacked.mat');
tau_stacked_11 = tau_stacked_11.tau_stacked_11;
tau_stacked_12 = load('tau12_stacked.mat');
tau_stacked_12 = tau_stacked_12.tau_stacked_12;
tau_stacked_13 = load('tau13_stacked.mat');
tau_stacked_13 = tau_stacked_13.tau_stacked_13;
tau_stacked_14 = load('tau14_stacked.mat');
tau_stacked_14 = tau_stacked_14.tau_stacked_14;
tau_stacked_21 = load('tau21_stacked.mat');
tau_stacked_21 = tau_stacked_21.tau_stacked_21;
tau_stacked_22 = load('tau22_stacked.mat');
tau_stacked_22 = tau_stacked_22.tau_stacked_22;
tau_stacked_23 = load('tau23_stacked.mat');
tau_stacked_23 = tau_stacked_23.tau_stacked_23;
tau_stacked_24 = load('tau24_stacked.mat');
tau_stacked_24 = tau_stacked_24.tau_stacked_24;

Y_stacked = cat(1, Y_stacked_11, Y_stacked_12, Y_stacked_13, Y_stacked_14, Y_stacked_21, Y_stacked_22, Y_stacked_23, Y_stacked_24);
tau_stacked = cat(1, tau_stacked_11, tau_stacked_12, tau_stacked_13, tau_stacked_14, tau_stacked_21, tau_stacked_22, tau_stacked_23, tau_stacked_24);

%% Get full column rank numerical regressor and dynamic coefficients estimation
disp('7');
[Y_rref, li_cols] = rref(Y_stacked);          % Gauss-Jordan elimination       
Yr_stacked = Y_stacked(:, li_cols);           % reduced numeric stacked regressor
Yr_stacked_pinv = pinv(Yr_stacked);           % pseudoinverted numerical reduced stacked regressor
pi_r_hat = Yr_stacked_pinv*tau_stacked;       % dynamic coefficients estimate (numerical)  (HERE FOR NUMERICAL VALUE COEFF.)
save('pi_r_hat.mat', 'pi_r_hat');

%% Get symbolic form of dynamic coefficients
disp('8');
Y_rref(abs(Y_rref)<1e-10) = 0;       % set to zero coefficients smaller than threshold
Yr = Y(:, li_cols);                  % reduced symbolic regressor (only linearly independent columns)
save('Yr.mat', 'Yr'); 
pi_r = Y_rref*p_mod;   
pi_r_sym = pi_r(1:length(li_cols));  % symbolic form of the dynamic coefficients (HERE FOR SYMBOLIC FORM COEFF)
save('pi_r_sym.mat', 'pi_r_sym');

%% Error computation
disp('9');
pi_r_sym = load('pi_r_sym.mat');
pi_r_sym = pi_r_sym.pi_r_sym;
pi_r_hat = load('pi_r_hat.mat');
pi_r_hat = pi_r_hat.pi_r_hat;
pi_r_reconstructed = double(subs(pi_r_sym, p_mod.', cat(2, masses_values, ctm_values, inertia_values)));
error = pi_r_reconstructed-pi_r_hat;
save('pi_r_recons.mat', 'pi_r_reconstructed');
save('error.mat', 'error');

%% Dynamic coefficients validation

[tau_est, tau_real, timesteps] = coeff_validation(pi_r_hat, pi_r_reconstructed, Yr, cat(2, params_joint_sym, p.', dh_sym, g0) , cat(2, masses_values, com_values, inertia_values, dh_values, 9.81));

%%
[tau_est1, tau_real_1, timesteps_1] = valid1(pi_r_hat, pi_r_recon, Yr, param_symbols, param_values);
[tau_est2, tau_real_2, timesteps_2] = valid1(pi_r_hat, pi_r_recon, Yr, param_symbols, param_values);
[tau_est3, tau_real_3, timesteps_3] = valid1(pi_r_hat, pi_r_recon, Yr, param_symbols, param_values);
[tau_est4, tau_real_4, timesteps_4] = valid1(pi_r_hat, pi_r_recon, Yr, param_symbols, param_values);

%%
tau_est1 = load('tau_est1.mat');
tau_est1=tau_est1.tau_est1;
tau_est2 = load('tau_est2.mat');
tau_est2=tau_est2.tau_est2;
tau_est3 = load('tau_est3.mat');
tau_est3=tau_est3.tau_est3;
tau_est4 = load('tau_est4.mat');
tau_est4=tau_est4.tau_est4;

tau_real1 = load('tau_real1.mat');
tau_real1=tau_real1.tau_real1;
tau_real2 = load('tau_real2.mat');
tau_real2=tau_real2.tau_real2;
tau_real3 = load('tau_real3.mat');
tau_real3=tau_real3.tau_real3;
tau_real4 = load('tau_real4.mat');
tau_real4=tau_real4.tau_real4;

tau_est = cat(1, tau_est1, tau_est2, tau_est3, tau_est4);
tau_real = cat(1, tau_real1, tau_real2, tau_real3, tau_real4);
%% Plot torques estimated and obtained from dynamic model
timesteps = 0:0.025:10-0.025;
duration = 10;
margin = 0.005;
figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Torques comparison', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(timesteps, tau_est(j:7:length(tau_est)), 'r', timesteps, tau_real(j:7:length(tau_real)), 'b:');
	grid on
    xlabel('t [s]')
	ylabel(strcat('tau_{',num2str(j),'} [Nm]'))
    xlim([-margin, duration+margin])
    ylim([min(min(tau_est(j:7:length(tau_est))), min(tau_real(j:7:length(tau_real))))-margin, max(max(tau_est(j:7:length(tau_est))), max(tau_real(j:7:length(tau_real))))+margin])
end
legend('Estimated', 'Reconstructed');