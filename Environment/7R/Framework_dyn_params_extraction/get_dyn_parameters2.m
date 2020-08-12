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

[LB, UB] = read_bounds('../Data/bounds/bounds_7R_normal.csv');
min_mass = 1;
max_mass = 30;

% %% Initialize parameters of the optimization algorithm
% 
% n_runs = 50;    % number of restart of the algorithm ERA 10
% n_iter = 35;    % successive runs algoritm (K) ERA 5
% 
% losses = Inf*ones(n_runs, 1);  % loss for each run
% dyn_params_est = zeros(n_joints*10, n_runs); % estimated dyn params for each run
% 
% %% Start simulation
% 
% disp('SIMULATION 7R_25');
% disp('');
% outputs = cell(n_runs, 1);    % variables needed by the algorithm
% exitflags = zeros(n_runs, 1); % variables needed by the algorithm
% tic
% for i=1:n_runs
%    for opt_step=1:n_iter
%        s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
%        disp(s);
%        if opt_step==1       % if first iteration then get initial state
%         x0 = rand(n_joints*10, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
%        end
%        options = saoptimset('HybridFcn', {@fmincon}); % use Nelder-Mead optimization as hybrid function
%        [X,fval,exitflag,output] = simulannealbnd(@(x) get_loss(x),x0,LB,UB,options);   % step of simulated annealing
%        x0 = X;      % update the state
%    end
% losses(i) = fval;
% dyn_params_est(:, i) = X;
% outputs = output;
% exitflags(i) = exitflag;
% end
% toc
% 
% %% Retrieve optimal solution
% 
% min_loss = find(losses==min(losses));          % get lowest loss
% dyn_parameters = dyn_params_est(:, min_loss);  % get opt dyn params
% 
% save('../Data/results_sim/normal/7R_25_loss.mat', 'min_loss');
% save('../Data/results_sim/normal/7R_25_params.mat', 'dyn_parameters');

% Initialize parameters of the optimization algorithm

n_runs = 50;    % number of restart of the algorithm ERA 10
n_iter = 35;    % successive runs algoritm (K) ERA 5

losses = Inf*ones(n_runs, 1);  % loss for each run
dyn_params_est = zeros(n_joints*10, n_runs); % estimated dyn params for each run

%% Start simulation

disp('SIMULATION 7R_28');
disp('');
outputs = cell(n_runs, 1);    % variables needed by the algorithm
exitflags = zeros(n_runs, 1); % variables needed by the algorithm
tic
for i=1:n_runs
   for opt_step=1:n_iter
       s = sprintf('Run %d of %d, step %d of %d',i,n_runs,opt_step,n_iter);
       disp(s);
       if opt_step==1       % if first iteration then get initial state
        x0 = rand(n_joints*10, 1).*(UB-LB) + LB;  % uniform random value of initial parameters
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

save('../Data/results_sim/normal/7R_28_loss.mat', 'min_loss');
save('../Data/results_sim/normal/7R_28_params.mat', 'dyn_parameters');