% @author Giada Simionato, 1822614.
% Function that returns the estimated torques (for each sampling instant) from the estimate of the
% dynamic coefficients and a sampled sinusoidal trajectory and the real
% obtained from the dynamic model.
% :return tau_estimate: estimate of the torques given the current estimate of the coefficients
% :return tau_model: torques computed from dynamic model

function [tau_estimate, tau_model, time_instants] = valid1(pi_r_hat, pi_r_recon, Yr, joint_symbols, Q, dQ, ddQ)

%% Trajectory parameters definition


n_joints = size(Yr, 1);
discard = true;

sampling_rate = 0.025;  % sampling rate trajectory [s]
duration = 10;        % duration of trajectory [s]
margin = 1;           % margin [s]

q_max = [2.9670, 2.0943, 2.9670, 2.0943, 2.9670, 2.0943, 2.9670];   % max joint positions [rad]
dq_max = [1.7453, 1.9199, 1.7453, 2.2689, 2.2689, 3.1416, 3.1416];  % max joint velocities [rad/s]
ddq_max = [5.236, 5.236, 5.236, 5.236, 5.236, 5.236, 5.236];        % max joint accelerations [rad/s^2]

Y_r_stacked = [];

%% Torques estimation

time_instants = size(Q, 2);

for t_inst=1:time_instants
    fprintf('Sample %d of %d \n', t_inst, time_instants);
    qk_sample = Q(:, t_inst)';   % [qk_t1, qk_t2, qk_t3]
    dqk_sample = dQ(:, t_inst)';
    ddqk_sample = ddQ(:, t_inst)';
    param_values_aug = cat(2, qk_sample, dqk_sample, ddqk_sample);
    
    Yr_k = double(subs(Yr, joint_symbols, param_values_aug));
    Y_r_stacked = cat(1, Y_r_stacked, Yr_k);

end
tau_estimate = Y_r_stacked*pi_r_hat;
tau_model = Y_r_stacked*pi_r_recon;
end