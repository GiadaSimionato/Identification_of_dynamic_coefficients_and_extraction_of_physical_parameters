%function [fm_real, fm_feasible, fm_unfeasible] = get_behaviour(fm_real_value, fm_feasible_value, fm_unfeasible_value, joint_params_sym, qk, dqk, ddqk, time_instants)
function [fm_0_f, fm_1_f, fm_2_f] = get_behaviour3(fm_0, fm_1, fm_2, joint_params_sym, qk, dqk, ddqk, time_instants)

%     fm_real = [];
%     fm_feasible = [];
%     fm_unfeasible = [];
fm_0_f = [];
fm_1_f = [];
fm_2_f = [];
    
    for i=1:length(time_instants)
        fprintf('Sample %d of %d \n', i, length(time_instants));
        sample = cat(2, qk(:,i).', dqk(:,i).', ddqk(:,i).');
        fm_0_k = double(subs(fm_0, joint_params_sym, sample));
        fm_1_k = double(subs(fm_1, joint_params_sym, sample));
        fm_2_k = double(subs(fm_2, joint_params_sym, sample));

%         fm_rk = double(subs(fm_real_value, joint_params_sym, sample));
%         fm_fk = double(subs(fm_feasible_value, joint_params_sym, sample));
%         fm_uk = double(subs(fm_unfeasible_value, joint_params_sym, sample));
%         fm_real = cat(3, fm_real, fm_rk);
%         fm_feasible = cat(3, fm_feasible, fm_fk);
%         fm_unfeasible = cat(3, fm_unfeasible, fm_uk);
        fm_0_f = cat(3, fm_0_f, fm_0_k);
        fm_1_f = cat(3, fm_1_f, fm_1_k);
        fm_2_f = cat(3, fm_2_f, fm_2_k);

    end
end