function [error_percent_change,v_min_vals] = random_search_obj_func(obj_func,p_opt)
    
s_eq = 153.9/21;

V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));

error_opt = obj_func(p_opt);

num_samples = 1500;

sample_params = zeros(num_samples,length(p_opt));

param_std_var = .2;
param_std_var = param_std_var.*p_opt;
error_vals_sweep = zeros(num_samples,1);
v_vals = zeros(num_samples,1);
disp('Beginning sampling...')
for n=1:num_samples
    sample_param = p_opt + randn(1,length(p_opt)).*param_std_var;
    sample_params(n,:) = sample_param;
    
    error_val = obj_func(sample_param);
    v_val = V(sample_param,s_eq);
    
    error_vals_sweep(n) = error_val;
    v_vals(n) = v_val;
end
disp('sampling finished')

error_diffs_sweep = error_vals_sweep-error_opt;
error_data = [sample_params,error_diffs_sweep,v_vals];
error_data_sorted = sortrows(error_data,6);

error_diff_vals = linspace(min(error_diffs_sweep),min(error_diffs_sweep)+error_opt,25);

v_min_vals = zeros(size(error_diff_vals));

for i=1:20
    delta = error_diff_vals(i);
    min_v = inf;
    for n=1:length(error_data_sorted)
        if(error_data_sorted(n,6) <= delta)
            v_val = error_data_sorted(n,7);
            if(v_val < min_v)
                min_v = v_val;
            end

        end

    end

    v_min_vals(i) = min_v;

end

disp('Finished sorting')
%% Plotting:

error_percent_change = (error_diff_vals/error_opt)*100;

figure()
plot(error_percent_change,v_min_vals,'.-','LineWidth',5,'MarkerSize',40)
set(gca,'FontSize',30)
ylabel('V_{eq}')
xlabel('\delta percent change in error')
title('Robustness tradeoff')
xlim([0,100])
end

