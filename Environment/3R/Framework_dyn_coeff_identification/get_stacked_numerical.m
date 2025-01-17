% @author Giada Simionato, 1822614.
% Function that returns the numeric stacked regressor (or torque) based on samples of trajectories and benchmark values for dynamic parameters.

function Y_stacked=get_stacked_numerical(Y, q_k, dq_k, ddq_k, joint_symbols)

n_joints = size(q_k, 1);         % number of joints
n_timesteps = size(q_k, 2);      % number of samples for each trajectory
n_traj = size(q_k, 3);           % number of trajectories

q_sample = zeros(n_joints, 1);
dq_sample = zeros(n_joints, 1);
ddq_sample = zeros(n_joints, 1);
Y_stacked = [];

for traj=1:n_traj
    s = sprintf('Processing trajectory %d of %d', traj, n_traj);
    disp(s);
    for sample=1:n_timesteps
        s = sprintf('Sample %d of %d', sample, n_timesteps);
        disp(s);
        q_sample = q_k(:, sample, traj);       % [q1kt, q2kt, q3kt]'
        dq_sample = dq_k(:, sample, traj);     % [dq1kt, dq2kt, dq3kt]'
        ddq_sample = ddq_k(:, sample, traj);   % [ddq1kt, ddq2kt, ddq3kt]'
        params_values_aug = cat(2, q_sample', dq_sample', ddq_sample');
        Y_k = double(subs(Y, joint_symbols, params_values_aug));      % numerical version
        Y_stacked = cat(1, Y_stacked, Y_k);    % stack results of samples
         
    end
end
end