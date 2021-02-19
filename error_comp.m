veh_V_full = readmatrix('vehV.csv');
veh_X_full = readmatrix('vehX.csv');
veh_S_full = readmatrix('vehS.csv');
veh_T_full = readmatrix('vehT.csv');
veh_VL_full = readmatrix('vehVL.csv');
%% Select portion where humans are driving:

lower_index = 1000;
upper_index = 8000;

veh_V = veh_V_full(:,lower_index:upper_index);
veh_T = veh_T_full(:,lower_index:upper_index);
veh_S = veh_S_full(:,lower_index:upper_index);
veh_VL = veh_VL_full(:,lower_index:upper_index);
veh_X = veh_X_full(:,lower_index:upper_index);
%% Model and error func:

V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));
accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));

error_string = 'spacing';
obj_func = @(p) ARED_rmse(p,accel_func,veh_T,veh_S,veh_V,veh_VL,error_string);

%% sample in a normalize ball around the optimal:
p_opt = [0.6660,21.5975,8.9368,2.2146,2.8150];

s_eq = 153.9/21;

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
%% sort wrt to how 'far away' the params are:
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


%% plot results:
figure()
plot(v_vals,error_vals_sweep,'.','MarkerSize',20)

%% Look at values surrounding the local optima:

s_eq = 153.9/21;

a=0.6660;
b=21.5975;
d_star=2.8150;

Vm_vals = linspace(5,15,100);
s0_vals = linspace(0.5,3.0,50);

[VM,S0] = meshgrid(Vm_vals,s0_vals);

error_vals_sweep = zeros(size(VM));

max_speed_vals = zeros(size(VM));

for i=1:length(s0_vals)
    for j=1:length(Vm_vals)
        Vm = VM(i,j);
        s0 = S0(i,j);
        p = [a,b,Vm,s0,d_star];
        error_vals_sweep(i,j) = obj_func(p);
        max_speed_vals(i,j) = V(p,s_eq);
    end
end
disp('Param sweep finished')
%%      
error_diffs_sweep = error_vals_sweep-error_opt;

figure()
contourf(VM,S0,error_diffs_sweep)
colorbar()
title('Error differences')
ylabel('s0')
xlabel('Vm')
set(gca,'FontSize',24)
hold on
plot(8.9368,2.2146,'ro','MarkerSize',15)
plot(8.9368,2.2146,'r.','MarkerSize',30)

figure()
contourf(VM,S0,max_speed_vals)
colorbar()
title('Estimate of max speed')
ylabel('s0')
xlabel('Vm')
set(gca,'FontSize',24)
hold on
plot(8.9368,2.2146,'ro','MarkerSize',15)
plot(8.9368,2.2146,'r.','MarkerSize',30)
hold on
contour(VM,S0,error_diffs_sweep,[5,5],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs_sweep,[1,1],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs_sweep,[.5,.5],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs_sweep,[.25,.25],'k--','ShowText','on','LineWidth',5)


% figure()
% scatter(reshape(max_speed_vals,numel(max_speed_vals),1),reshape(errof_diffs,numel(errof_diffs),1))
% title('Errors vs. max speeds')
% ylabel('Error Difference')
% xlabel('Equilbrium Speed')
% set(gca,'FontSize',24)
% ylim([0,max(error_vals,[],'all')])