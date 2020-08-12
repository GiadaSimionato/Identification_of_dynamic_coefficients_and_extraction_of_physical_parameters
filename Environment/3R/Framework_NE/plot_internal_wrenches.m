% @author Giada Simionato, 1822614.
% Script that plots internal forces in 3R manipulator.

clear all
close all
clc

CurPath = pwd();
addpath([CurPath, '/../Framework_dyn_params_extraction']);
addpath([CurPath, '/../Data']);

%% Symbolic dynamic parameters initialization

syms g0 q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3...
    m1 m2 m3 a2 a3...
    J1xx J1xy J1xz J1yy J1yz J1zz ...
    J2xx J2xy J2xz J2yy J2yz J2zz ...
    J3xx J3xy J3xz J3yy J3yz J3zz ...
    c1x c1y c1z c2x c2y c2z c3x c3y c3z real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];

p1 = [m1; m2; m3];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z];
p3 = [J1; J2; J3];

p = [p1; p2; p3];        % vector of symbolic dynamic parameters
p = p.';

q = [q1, q2, q3];
dq = [dq1, dq2, dq3];
ddq = [ddq1, ddq2, ddq3]; 
dh_sym = [a2, a3];      % vector of kinematic parameters

joint_params_sym = cat(2, q, dq, ddq);
fixed_params = cat(2, dh_sym, g0);

%% Real dynamic parameters

masses_values = [3.5, 0.8, 1.2];  % values for m1, m2, m3 [kg]
com_values = [1e-03, -12e-02, 2e-03, -88e-03, 1e-03, 2e-03, -6e-02, 2e-03, 3e-03];  % values for c1x c1y c1z c2x c2y c2z c3x c3y c3z [m]
inertia_values = [5.64e-02, 3.15e-04, -1.4e-05, 2.63e-02, 6.3e-04, 7.98e-02, 4.98e-04, 1.27e-04, 2.54e-04, 1.34e-02, -3.2e-06, 1.34e-02, 5.71e-04, 2.52e-04, 3.78e-04, 9.02e-03, -1.44e-05, 9.01e-03]; % values for J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz [kg*m^2]
dh_values = [0.22, 0.15]; % values for a2, a3 [m]

p_values_real = cat(2, masses_values, com_values, inertia_values);
fixed_values = cat(2, dh_values, 9.81);

%% Extracted dynamic parameters

p_1 = load('../Data/results_sim/normal/3R_1_params.mat');
p_1 = p_1.dyn_parameters.';
p_4 = load('../Data/results_sim/normal/3R_4_params.mat');
p_4 = p_4.dyn_parameters.';
p_5 = load('../Data/results_sim/tight/3R_5_params.mat');
p_5 = p_5.dyn_parameters.';
p_8 = load('../Data/results_sim/tight/3R_8_params.mat');
p_8 = p_8.dyn_parameters.';
p_9 = load('../Data/results_sim/loose/3R_9_params.mat');
p_9 = p_9.dyn_parameters.';
p_12 = load('../Data/results_sim/loose/3R_12_params.mat');
p_12 = p_12.dyn_parameters.';

%% NE algorithm

[tau, forces_momenta] = NE();
tau = expand(tau);
forces_momenta = expand(forces_momenta);

%% Customize forces_momenta

fm_fixed_value = subs(forces_momenta, fixed_params, fixed_values);

fm_real_value = subs(fm_fixed_value, p, p_values_real);
fm_feasible_value = subs(fm_fixed_value, p, p_values_feasible);
fm_unfeasible_value = subs(fm_fixed_value, p, p_values_unfeasible);

fm_real_value = subs(fm_fixed_value, p, p_values_real);

fm_1 = subs(fm_fixed_value, p, p_1);
fm_4 = subs(fm_fixed_value, p, p_4);
fm_5 = subs(fm_fixed_value, p, p_5);
fm_8 = subs(fm_fixed_value, p, p_8);
fm_9 = subs(fm_fixed_value, p, p_9);
fm_12 = subs(fm_fixed_value, p, p_12);

%% Get sinusoidal trajectory

[qk, dqk, ddqk, time_instants] = get_sinusoidal_traj();

%%

[fm_real, fm_1, fm_4, fm_5, fm_8, fm_9, fm_12] = get_behaviour(fm_real_value, fm_1, fm_4, fm_5, fm_8, fm_9, fm_12, joint_params_sym, qk, dqk, ddqk, time_instants);

%% Get norm
n_joints = 3;

forces_vect_real = zeros(n_joints, length(time_instants));
forces_vect_feasible = zeros(n_joints, length(time_instants));
forces_vect_unfeasible = zeros(n_joints, length(time_instants));

for j=1:n_joints
    for i=1:length(time_instants)
        s = sprintf('Joint %d: samples %d of %d', j, i, length(time_instants));
        disp(s);
        forces_vect_real(j, i) = norm(fm_real(1:3, j, i));
        forces_vect_feasible(j, i) = norm(fm_4(1:3, j, i));
        forces_vect_unfeasible(j, i) = norm(fm_1(1:3, j, i));
    end
end

%% Plot

duration = 20;
margin = 0.005;
margin_vert = 3;
figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Internal forces comparison', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(3,1,j)
	plot(time_instants, forces_vect_real(j, :), 'r', time_instants, forces_vect_feasible(j, :), 'b:', time_instants, forces_vect_unfeasible(j, :), 'g--');
	grid on
    xlabel('t [s]')
	ylabel(strcat('intForces_{',num2str(j),'} [N]'))
    xlim([-margin, duration+margin])
    ylim([min(min(min(forces_vect_real(j,:)), min(forces_vect_feasible(j, :))), min(forces_vect_unfeasible(j, :)))-margin_vert, max(max(max(forces_vect_real(j,:)), max(forces_vect_feasible(j, :))), max(forces_vect_unfeasible(j, :)))+margin_vert])
end
legend('Real', 'Feasible', 'Unfeasible');