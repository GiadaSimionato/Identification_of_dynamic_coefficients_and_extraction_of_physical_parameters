% @author Giada Simionato, 1822614.
% Function that automatically generates exciting trajectories.

function [q_stacked, dq_stacked, ddq_stacked] = generate_exc_traj(n_traj)

if ~exist('../Data', 'dir')
   mkdir('../Data')
   mkdir('../Data/iden_traj')
end

%% Initialization trajectory parameters

wf = 0.15*pi;    % angular frequency
L = 5;           % number of harmonics
n_joints = 7;    % number of joints
n_traj_curr = 1; % current number of trajectories generated
s_rate = 0.034;   % sampling rate [s]  WAS 0.01
duration = 20;   % trajectory duration [s]
margin = 1.02;      % margin traj adjustment (vel. and accel.) [s]
margin_end = 19.992;

q_max = [2.9670, 2.0943, 2.9670, 2.0943, 2.9670, 2.0943, 2.9670];   % max joint positions [rad]
dq_max = [1.7453, 1.9199, 1.7453, 2.2689, 2.2689, 3.1416, 3.1416];  % max joint velocities [rad/s]
ddq_max = [5.236, 5.236, 5.236, 5.236, 5.236, 5.236, 5.236];        % max joint accelerations [rad/s^2]

syms t real
sampling_instants = 0:s_rate:duration;

q = zeros(n_joints, length(sampling_instants));   % will cont. values of position for each joint (row) for each sampling instant (cols)
dq = zeros(n_joints, length(sampling_instants));  % will cont. values of velocities for each joint (row) for each s. inst. (cols)
ddq = zeros(n_joints, length(sampling_instants)); % will cont. values of accelerations for each joint (row) for each s. inst. (cols)
A = zeros(n_joints, L); % will contain values param al,j for each joint (row) for each harmonic (cols)
B = zeros(n_joints, L); % same for param. bl,j
q0 = zeros(n_joints, 1);   % will contain values of initial traj value for each joint

q_stacked = zeros(n_joints, length(sampling_instants), n_traj);
dq_stacked = zeros(n_joints, length(sampling_instants), n_traj);
ddq_stacked = zeros(n_joints, length(sampling_instants), n_traj);
A_stacked = zeros(n_joints, L, n_traj);
B_stacked = zeros(n_joints, L, n_traj);
q0_stacked = zeros(n_joints, n_traj);

while n_traj_curr <= n_traj
    
%% Trajectory parameters computation

A = - rand(size(A)) + 0.5;  % a_jl parameters trajectory (vary between -1, 1)
B = - rand(size(B)) + 0.5;  % b_jl parameters trajectory (vary between -1, 1)
q0 = -0.5 +rand(size(q0)); 


aux_sin = sin(wf*t*[1:L])./[1:L];    % auxiliary vectors for traj. behaviour computation
aux_cos = cos(wf*t*[1:L])./[1:L];    % row vectors
aux_sin_der = sin(wf*t*[1:L]);
aux_cos_der = cos(wf*t*[1:L]);
aux_sin_dder = [1:L].*sin(wf*t*[1:L]);
aux_cos_dder = [1:L].*cos(wf*t*[1:L]);

%% Trajectory behaviour computation

% Joint positions computation

for j=1:n_joints
	q_j = simplify((1/wf)*(A(j,:)*aux_sin' - B(j,:)*aux_cos') + q0(j)); % trajectory for joint positions
    for t_i=1:length(sampling_instants)
        instant = sampling_instants(t_i);
        q(j,t_i) = double(subs(q_j, instant));
    end
end

% Joint velocities and accelerations computation (central behaviour)

for j=1:n_joints
    dq_j = A(j,:)*aux_cos_der' + B(j, :)*aux_sin_der';                    % trajectory for joint velocities
    ddq_j = wf*(-A(j,:)*aux_sin_dder' + B(j,:)*aux_cos_dder');            % trajectory for joint accelerations
    for t_i=int32(margin/s_rate):int32((duration-margin)/s_rate)          % dopo estremo primo -1 e est secondo +1
        instant = sampling_instants(t_i);
        dq(j,t_i) = double(subs(dq_j, instant));
        ddq(j,t_i) = double(subs(ddq_j, instant));
    end
end

% Joint velocities and accelerations computation (side behaviour)

for j=1:n_joints
    dq_j_init = dq(j, find(sampling_instants==margin))*sin(4*wf*t)/sin(wf*margin);
    ddq_j_init = ddq(j, find(sampling_instants==margin))*(1-cos(wf*t))/(1-cos(wf*margin));
    dq_j_end = -dq(j, find(sampling_instants==margin_end)-1)*sin(wf*(t-duration))/sin(wf*margin);
    ddq_j_end = ddq(j, find(sampling_instants==margin_end)-1)*(1-cos(wf*(t-duration)))/(1-cos(wf*margin));

    for t_i=1:int32(margin/s_rate)-1
        instant = sampling_instants(t_i);
        dq(j, t_i) = double(subs(dq_j_init, instant));
        ddq(j, t_i) = double(subs(ddq_j_init, instant));
    end
    for t_i=int32((duration-margin)/s_rate)+1:length(sampling_instants)
        instant = sampling_instants(t_i);
        dq(j, t_i) = double(subs(dq_j_end, instant));
        ddq(j, t_i) = double(subs(ddq_j_end, instant));
    end
end

%% Plot trajectories

figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint positions', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(sampling_instants, q(j,:), 'b', sampling_instants, repelem(q_max(j), length(sampling_instants)), 'r', sampling_instants, repelem(-q_max(j), length(sampling_instants)), 'r');
	grid on
    xlabel('t [s]')
	ylabel(strcat('q_{',num2str(j),'} [rad]'))
    xlim([-margin, duration+margin])
    ylim([min(min(q(j, :)), -q_max(j))-margin, max(max(q(j,:)), q_max(j))+margin])
end

figure(2)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint velocities', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(sampling_instants, dq(j,:), 'b', sampling_instants, repelem(dq_max(j), length(sampling_instants)), 'r', sampling_instants, repelem(-dq_max(j), length(sampling_instants)), 'r');
	grid on
    xlabel('t [s]')
	ylabel(strcat('dq_{',num2str(j),'} [rad/s]'))
    xlim([-margin, duration+margin])
    ylim([min(min(dq(j, :)), -dq_max(j))-margin, max(max(dq(j,:)), dq_max(j))+margin])
end

figure(3)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint accelerations', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(sampling_instants, ddq(j,:), 'b', sampling_instants, repelem(ddq_max(j), length(sampling_instants)), 'r', sampling_instants, repelem(-ddq_max(j), length(sampling_instants)), 'r');
	grid on
    xlabel('t [s]')
	ylabel(strcat('ddq_{',num2str(j),'} [rad/s^2]'))
    xlim([-margin, duration+margin])
    ylim([min(min(ddq(j, :)), -ddq_max(j))-margin, max(max(ddq(j,:)), ddq_max(j))+margin])
end

%% Save trajectories

prompt = 'Do you want to keep and save this trajectory? [y/n]: ';
str = input(prompt,'s');
if strcmp('y',str)
    q_stacked(:, :, n_traj_curr) = q;
    dq_stacked(:, :, n_traj_curr) = dq;
    ddq_stacked(:, :, n_traj_curr) = ddq;
    A_stacked(:, :, n_traj_curr) = A;
    B_stacked(:, :, n_traj_curr) = B;
    q0_stacked(:, n_traj_curr) = q0;
    save('../Data/iden_traj/q_stacked.mat', 'q_stacked');
    save('../Data/iden_traj/dq_stacked.mat', 'dq_stacked');
    save('../Data/iden_traj/ddq_stacked.mat', 'ddq_stacked');
    save('../Data/iden_traj/A_stacked.mat', 'A_stacked');
    save('../Data/iden_traj/B_stacked.mat', 'B_stacked');
    save('../Data/iden_traj/q0_stacked.mat', 'q0_stacked');
    n_traj_curr = n_traj_curr + 1;
    close all
end
end
end
