%% Model and error func:

V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));
accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));

error_string = 'spacing';
obj_func = @(p) ARED_rmse(p,accel_func,veh_T,veh_S,veh_V,veh_VL,error_string);

%% sample in a normalize ball around the optimal:
p_opt = [0.6660,21.5975,8.9368,2.2146,2.8150];

s_eq = 153.9/21;

num_samples = 500;
param_std_var = .2;
param_std_var = param_std_var.*p_opt;
error_vals = zeros(num_samples,1);
v_vals = zeros(num_samples,1);

for n=1:num_samples
    sample_param = p_opt + randn(1,length(p_opt)).*param_std_var;
    error_val = obj_func(sample_param);
    v_val = V(sample_param,s_eq);
    
    error_vals(n) = error_val;
    v_vals(n) = v_val;
end

%% plot results:
figure()
plot(v_vals,error_vals,'.','MarkerSize',20)

%% Look at values surrounding the local optima:

s_eq = 153.9/21;

a=0.6660;
b=21.5975;
d_star=2.8150;

Vm_vals = linspace(5,15,100);
s0_vals = linspace(0.5,3.0,50);

[VM,S0] = meshgrid(Vm_vals,s0_vals);

error_vals = zeros(size(VM));

max_speed_vals = zeros(size(VM));

for i=1:length(s0_vals)
    for j=1:length(Vm_vals)
        Vm = VM(i,j);
        s0 = S0(i,j);
        p = [a,b,Vm,s0,d_star];
        error_vals(i,j) = obj_func(p);
        max_speed_vals(i,j) = V(p,s_eq);
    end
end
disp('Param sweep finished')
%%      
error_diffs = error_vals-error_opt;

figure()
contourf(VM,S0,error_diffs)
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
contour(VM,S0,error_diffs,[5,5],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs,[1,1],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs,[.5,.5],'k--','ShowText','on','LineWidth',5)
contour(VM,S0,error_diffs,[.25,.25],'k--','ShowText','on','LineWidth',5)


% figure()
% scatter(reshape(max_speed_vals,numel(max_speed_vals),1),reshape(errof_diffs,numel(errof_diffs),1))
% title('Errors vs. max speeds')
% ylabel('Error Difference')
% xlabel('Equilbrium Speed')
% set(gca,'FontSize',24)
% ylim([0,max(error_vals,[],'all')])