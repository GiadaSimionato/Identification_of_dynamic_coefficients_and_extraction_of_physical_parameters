% @author Giada Simionato, 1822614.
% Script that automatically generates sinusoidal trajectories.

function [q, dq, ddq, time_instants] = get_sinusoidal_traj()

n_joints = 7;
discard = true;
sampling_rate = 0.1;  % sampling rate trajectory [s]
duration = 20;        % duration of trajectory [s]
margin = 1;           % margin [s]

time_instants = 0:sampling_rate:duration;  % sampling instants

q_max = [2.9670, 2.0943, 2.9670, 2.0943, 2.9670, 2.0943, 2.9670];   % max joint positions [rad]
dq_max = [1.7453, 1.9199, 1.7453, 2.2689, 2.2689, 3.1416, 3.1416];  % max joint velocities [rad/s]
ddq_max = [5.236, 5.236, 5.236, 5.236, 5.236, 5.236, 5.236];        % max joint accelerations [rad/s^2]

Amax = dq_max;
Amin = 0;
Tmin = 2*pi.*dq_max./ddq_max;
Tmax = 2*pi.*q_max./dq_max;

q = zeros(n_joints,length(time_instants));
dq = zeros(n_joints,length(time_instants));
ddq = zeros(n_joints,length(time_instants));

while(discard)
    
A = rand(1, n_joints).*Amax;
T = Tmin + (Tmax - Tmin).*rand(1, n_joints);
q0 = zeros(1, n_joints);
    
for j=1:n_joints

q(j,:) = -T(j)*A(j)*cos(2*pi*time_instants/T(j))/(2*pi) + q0(j);
dq(j,:) = A(j)*sin(2*pi*time_instants/T(j));
ddq(j, :) = 2*pi*A(j)*cos(2*pi*time_instants/T(j))/T(j);

end

figure(1)
annotation('textbox', [0 0.9 1 0.1], 'String', 'Joint positions', 'EdgeColor', 'none','HorizontalAlignment', 'center')
for j=1:n_joints
    subplot(4,2,j)
	plot(time_instants, q(j,:), 'b', time_instants, repelem(q_max(j), length(time_instants)), 'r', time_instants, repelem(-q_max(j), length(time_instants)), 'r');
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
	plot(time_instants, dq(j,:), 'b', time_instants, repelem(dq_max(j), length(time_instants)), 'r', time_instants, repelem(-dq_max(j), length(time_instants)), 'r');
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
	plot(time_instants, ddq(j,:), 'b', time_instants, repelem(ddq_max(j), length(time_instants)), 'r', time_instants, repelem(-ddq_max(j), length(time_instants)), 'r');
	grid on
    xlabel('t [s]')
	ylabel(strcat('ddq_{',num2str(j),'} [rad/s^2]' ))
    xlim([-margin, duration+margin])
    ylim([min(min(ddq(j, :)), -ddq_max(j))-margin, max(max(ddq(j,:)), ddq_max(j))+margin])
end


prompt = 'Do you want to keep and save this trajectory? [y/n]: ';
str = input(prompt,'s');
if strcmp('y',str)
    discard = false;
    save('../Data/traj_final_NE/q.mat', 'q');
    save('../Data/traj_final_NE/dq.mat', 'dq');
    save('../Data/traj_final_NE/ddq.mat', 'ddq');
    save('../Data/traj_final_NE/time_instants.mat', 'time_instants');
    close all
end
end
end