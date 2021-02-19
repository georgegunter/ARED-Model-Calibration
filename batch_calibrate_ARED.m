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

%% Look at calibrating OV-FTL:

V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));
accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));
p0 = [.8,10,11.0,2.23,5]; %[alpha,beta,Vm,d0,d_star]
error_string = 'spacing';
obj_func = @(p) ARED_rmse(p,accel_func,veh_T,veh_X,veh_S,veh_V,veh_VL,error_string);


%% Run optimization:
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
tic
[p_opt,error_opt] = fminunc(obj_func,p0,options);
toc
% converges to p = [0.6660,21.5975,8.9368,2.2146,2.8150];

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


figure()
scatter(reshape(max_speed_vals,numel(max_speed_vals),1),reshape(errof_diffs,numel(errof_diffs),1))
title('Errors vs. max speeds')
ylabel('Error Difference')
xlabel('Equilbrium Speed')
set(gca,'FontSize',24)
ylim([0,max(error_vals,[],'all')])