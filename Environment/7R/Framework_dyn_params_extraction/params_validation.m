% @author Giada Simionato, 1822614.
% Function that returns the estimated torques (for each sampling instant) from the estimate of the
% dynamic coefficients and a sampled sinusoidal trajectory and the real
% obtained from the dynamic model.
% :return tau_estimate: estimate of the torques given the current estimate of the coefficients
% :return tau_model: torques computed from dynamic model

function [tau_estimate, tau_model, time_instants] = params_validation(pi_r_hat, pi_r_recon, Yr, param_symbols)

%% Trajectory parameters definition
n_joints = size(Yr, 1);
discard = true;

sampling_rate = 0.01;  % sampling rate trajectory [s]
duration = 10;        % duration of trajectory [s]
margin = 1;           % margin [s]

time_instants = 0:sampling_rate:duration;  % sampling instants

q_max = [2.9670, 2.0943, 2.9670, 2.0943, 2.9670, 2.0943, 2.9670];   % max joint positions [rad]
dq_max = [1.7453, 1.9199, 1.7453, 2.2689, 2.2689, 3.1416, 3.1416];  % max joint velocities [rad/s]
ddq_max = [5.236, 5.236, 5.236, 5.236, 5.236, 5.236, 5.236];        % max joint accelerations [rad/s^2]

Q = zeros(n_joints,length(time_instants));
dQ = zeros(n_joints,length(time_instants));
ddQ = zeros(n_joints,length(time_instants));

Y_r_stacked = [];
tau_stacked = [];

%% Trajectory generation

while(discard)
    
A = dq_max.*rand(1, n_joints);  % (row vector)
T = 2*pi*A./ddq_max;            % (row vector)
q0 = q_max - (T.*A)/(2*pi);       
    
for j=1:n_joints
    
Q(j,:) = -T(j)*A(j)*cos(2*pi*time_instants/T(j))/(2*pi) + q0(j);
dQ(j,:) = A(j)*sin(2*pi*time_instants/T(j));
ddQ(j, :) = 2*pi*A(j)*cos(2*pi*time_instants/T(j))/T(j);

end

figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint positions', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(time_instants, Q(j,:))
	grid on
    xlabel('t [s]')
	ylabel(strcat('q_{',num2str(j),'} [rad]'))
    xlim([-margin, duration+margin])
    ylim([min(Q(j, :))-margin, max(Q(j,:))+margin])
end

figure(2)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint velocities', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(time_instants, dQ(j,:))
	grid on
    xlabel('t [s]')
	ylabel(strcat('dq_{',num2str(j),'} [rad/s]'))
    xlim([-margin, duration+margin])
    ylim([min(dQ(j, :))-margin, max(dQ(j,:))+margin])
end

figure(3)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint accelerations', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(time_instants, ddQ(j,:))
	grid on
    xlabel('t [s]')
	ylabel(strcat('ddq_{',num2str(j),'} [rad/s^2]'))
    xlim([-margin, duration+margin])
    ylim([min(ddQ(j, :))-margin, max(ddQ(j,:))+margin])
end

prompt = 'Do you want to keep and save this trajectory? [y/n]: ';
str = input(prompt,'s');
if strcmp('y',str)
    discard = false;
    close all
end
end

%% Torques estimation

for t_inst=1:length(time_instants)
    s = sprintf('Sample %d of %d', t_inst, length(time_instants));
    disp(s);
    qk_sample = Q(:, t_inst)';   % [qk_t1, qk_t2, qk_t3]
    dqk_sample = dQ(:, t_inst)';
    ddqk_sample = ddQ(:, t_inst)';
    param_values_aug = cat(2, qk_sample, dqk_sample, ddqk_sample);
    
    Yr_k = double(subs(Yr, param_symbols, param_values_aug));
    Y_r_stacked = cat(1, Y_r_stacked, Yr_k);
    
end

tau_estimate = Y_r_stacked*pi_r_hat;
tau_model = Y_r_stacked*pi_r_recon;
end
