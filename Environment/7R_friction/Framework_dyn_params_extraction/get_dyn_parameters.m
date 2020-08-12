%@author Giada Simionato, 1822614.

close all
clear all
clc

CurPath = pwd();
addpath([CurPath, '/../Data']);
addpath([CurPath, '/../Framework_dyn_coeff_identification']);

global opt_step
global min_mass
global max_mass
global LB
global UB

n_joints = 7;  % number of joints

%% Load bounds of dynamic parameters

[LB, UB] = read_bounds('../Data/bounds/bounds_7R_friction_normal.csv');
min_mass = 1;
max_mass = 30;

%% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_37');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

%% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/normal/7R_37_loss.mat', 'min_loss');
save('../Data/results_sim/normal/7R_37_params.mat', 'dyn_parameters');

% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

% Start simulation

disp('SIMULATION 7R_40');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss_full(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/normal/7R_40_loss.mat', 'min_loss');
save('../Data/results_sim/normal/7R_40_params.mat', 'dyn_parameters');

%% Load bounds of dynamic parameters

[LB, UB] = read_bounds('../Data/bounds/bounds_7R_friction_tight.csv');
min_mass = 13;
max_mass = 19;

%% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_41');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

%% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/tight/7R_41_loss.mat', 'min_loss');
save('../Data/results_sim/tight/7R_41_params.mat', 'dyn_parameters');

% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_44');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss_full(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

%% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/tight/7R_44_loss.mat', 'min_loss');
save('../Data/results_sim/tight/7R_44_params.mat', 'dyn_parameters');




[LB, UB] = read_bounds('../Data/bounds/bounds_7R_friction_loose.csv');
min_mass = 1;
max_mass = 100;

%% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_45');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

%% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/loose/7R_45_loss.mat', 'min_loss');
save('../Data/results_sim/loose/7R_45_params.mat', 'dyn_parameters');

%% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*13, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_48');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*13, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
       end
       options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
       [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss_full(x),x0,LB,UB,options);   % step of simulated annealing
       x0 = X;      % update the state
   end
losses(i) = fval;
dyn_params_est(:, i) = X;
outputs = output;
exitflags(i) = exitflag;
end
toc

%% Retrieve optimal solution

min_loss = find(losses==min(losses));          % get lowest loss
dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params

save('../Data/results_sim/loose/7R_48_loss.mat', 'min_loss');
save('../Data/results_sim/loose/7R_48_params.mat', 'dyn_parameters');

%% Validate extraction

syms m1 m2 m3 real                                  % init masses
syms c1x c1y c1z c2x c2y c2z c3x c3y c3z real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz real % init inertia tensors elems wrt RFi (assumption: real)
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real       % joint positions, velocities and accelerations
syms a2 a3 g0 real                                  % DH parameters
syms m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z real

J1 = [J1xx; J1xy; J1xz; J1yy; J1yz; J1zz];
J2 = [J2xx; J2xy; J2xz; J2yy; J2yz; J2zz];
J3 = [J3xx; J3xy; J3xz; J3yy; J3yz; J3zz];

p1 = [m1; m2; m3];
p2 = [c1x; c1y; c1z; c2x; c2y; c2z; c3x; c3y; c3z];
p3 = [J1; J2; J3];

p_sym = [p1; p2; p3];  

pi_r_hat = load('pi_r_hat.mat');    % identified coefficients (numerical)
pi_r_hat = pi_r_hat.pi_r_hat;

pi_r_sym = load('pi_r_sym');        % identified coefficients (symbolic)
pi_r_sym = pi_r_sym.pi_r_sym;

p = load('../Data/results_sim/loose/3R_12_params');
p = p.dyn_parameters;               % extracted parameters

pi_r_sym = subs(pi_r_sym, [m1c1x m1c1y m1c1z m2c2x m2c2y m2c2z m3c3x m3c3y m3c3z], [m1*c1x m1*c1y m1*c1z m2*c2x m2*c2y m2*c2z m3*c3x m3*c3y m3*c3z]);

pi_r_recons = double(subs(pi_r_sym, p_sym, p));   % coefficients reconstructed (from extracted parameters)
error = pi_r_hat-pi_r_recons;

%% Plot validation

dh_values = [0.22, 0.15]; % values for a2, a3 [m]

Yr = load('Yr.mat');
Yr = Yr.Yr;
Yr = subs(Yr, [a2 a3 g0], cat(2, dh_values, 9.81));
joint_params_sym = [q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3];
[tau_est, tau_model, t] = params_validation(pi_r_hat, pi_r_recons, Yr, joint_params_sym);

duration = 10;
margin = 0.005;
figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Torques comparison', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(3,1,j)
	plot(t, tau_est(j:3:length(tau_est)), 'r', t, tau_model(j:3:length(tau_model)), 'b:');
	grid on
    xlabel('t [s]')
	ylabel(strcat('tau_{',num2str(j),'} [Nm]'))
    xlim([-margin, duration+margin])
    ylim([min(min(tau_est(j:3:length(tau_est))), min(tau_model(j:3:length(tau_model))))-margin, max(max(tau_est(j:3:length(tau_est))), max(tau_model(j:3:length(tau_model))))+margin])
end
legend('Estimated', 'Reconstructed');

%% Check feasibility

disp('Checking feasibility...');
p_val = load('../Data/results_sim/loose/7R_48_params.mat');
p_val = p_val.dyn_parameters.';
decision = is_feasible(p_val);
disp(decision);