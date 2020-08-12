% @author: Giada Simionato, 1822614.
% Main script to recover the symbolic form of the dynamic model of a 3R
% manipulator.
% [Adapted from the code provided by Claudio Gaz.]

clear all
close all
clc

tau = get_tau_sym_3R(); % get symbolic dynamic model
save('dyn_mod_3R.mat', 'tau');

%% Save the dynamic model

% file = fopen('dyn_model_3R.txt', 'wt');
% fprintf(file, '%s\n', char(tau));
% fclose(file);