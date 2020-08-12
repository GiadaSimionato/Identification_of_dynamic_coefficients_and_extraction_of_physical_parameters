% @author Giada Simionato, 1822614.

function [fm_0_f, fm_1_f, fm_2_f, fm_3_f, fm_4_f, fm_5_f, fm_6_f] = get_behaviour(fm_0, fm_1, fm_2, fm_3, fm_4, fm_5, fm_6, joint_params_sym, qk, dqk, ddqk, time_instants)

fm_0_f = [];
fm_1_f = [];
fm_2_f = [];
fm_3_f = [];
fm_4_f = [];
fm_5_f = [];
fm_6_f = [];
    
    for i=1:length(time_instants)
        fprintf('Sample %d of %d \n', i, length(time_instants));
        sample = cat(2, qk(:,i).', dqk(:,i).', ddqk(:,i).');
        fm_0_k = double(subs(fm_0, joint_params_sym, sample));
        fm_1_k = double(subs(fm_1, joint_params_sym, sample));
        fm_2_k = double(subs(fm_2, joint_params_sym, sample));
        fm_3_k = double(subs(fm_3, joint_params_sym, sample));
        fm_4_k = double(subs(fm_4, joint_params_sym, sample));
        fm_5_k = double(subs(fm_5, joint_params_sym, sample));
        fm_6_k = double(subs(fm_6, joint_params_sym, sample));

        fm_0_f = cat(3, fm_0_f, fm_0_k);
        fm_1_f = cat(3, fm_1_f, fm_1_k);
        fm_2_f = cat(3, fm_2_f, fm_2_k);
        fm_3_f = cat(3, fm_3_f, fm_3_k);
        fm_4_f = cat(3, fm_4_f, fm_4_k);
        fm_5_f = cat(3, fm_5_f, fm_5_k);
        fm_6_f = cat(3, fm_6_f, fm_6_k);
    end
end