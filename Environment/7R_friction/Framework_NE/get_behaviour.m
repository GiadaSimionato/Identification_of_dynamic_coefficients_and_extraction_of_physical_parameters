%@author Giada Simionato, 1822614.

function [fm_0_f, fm_1_f] = get_behaviour(fm_0, fm_1, joint_params_sym, qk, dqk, ddqk, time_instants)

fm_0_f = [];
fm_1_f = [];
    
    for i=1:length(time_instants)
        fprintf('Sample %d of %d \n', i, length(time_instants));
        sample = cat(2, qk(:,i).', dqk(:,i).', ddqk(:,i).');
        fm_0_k = double(subs(fm_0, joint_params_sym, sample));
        fm_1_k = double(subs(fm_1, joint_params_sym, sample));

        fm_0_f = cat(3, fm_0_f, fm_0_k);
        fm_1_f = cat(3, fm_1_f, fm_1_k);
    end
end