veh_V_full = readmatrix('vehV.csv');
veh_X = readmatrix('vehX.csv');
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

%% Load library functions:
% Original OV-FTL function:
V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));
accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));
% make libraries for regression:

p_batch = [0.6660,21.5975,8.9368,2.2146,2.8150];

%Default values are picked so that can converge directly to what is found
%through batch calibration:
s0_sindy = p_batch(5);
s_star_sindy = p_batch(4);

V_sindy =  @(s) (tanh(s./s0_sindy-s_star_sindy)+tanh(s_star_sindy))/(1+tanh(s_star_sindy));  
FTL_sindy = @(s,ds_dt) (ds_dt./(s.^2));

%% Verify that these functions can recreate the other optimal model:
s_eq_vals = 1:.1:15;

V_diff = V_sindy(s_eq_vals)*p_batch(3) - V(p_batch,s_eq_vals);
figure()
plot(s_eq_vals,V_diff,'LineWidth',3)

s_vals = veh_S(1,:);
v_vals = veh_V(1,:);
vl_vals = veh_VL(1,:);
dsdt_vals = vl_vals-v_vals;

accel_opt_vals = accel_func(p_batch,s_vals,dsdt_vals,v_vals);
accel_sindy_vals = p_batch(1)*p_batch(3)*V_sindy(s_vals) - p_batch(1)*v_vals + p_batch(2)*FTL_sindy(s_vals,dsdt_vals);

accel_diff = accel_sindy_vals - accel_opt_vals;
figure()
plot(accel_diff)
hold on
plot(accel_opt_vals)

%% make accelerations:
DVDT = zeros(size(veh_V));
dt = 1/30;
for i=1:length(DVDT(:,1))
    v = veh_V(i,:);
    DVDT(i,:) = gradient(v,dt);
end

%% Load matrix to regress on:
y = reshape(DVDT,numel(DVDT),1);

s_vec = reshape(veh_S,numel(veh_S),1);
v_vec = reshape(veh_V,numel(veh_V),1);
vl_vec = reshape(veh_VL,numel(veh_VL),1);
dsdt_vec = vl_vec - v_vec;

A = [V_sindy(s_vec),v_vec,FTL_sindy(s_vec,dsdt_vec)]; % Make library for inversion

%% Perform regression:
tic
x = pinv(A)*y;
toc
disp('Regression finished')

%% extract parameters and 

alpha_sindy = -x(2);
beta_sindy = x(3);
vm_sindy = x(1)/alpha_sindy;

%% Calculate batch error:
p_sindy = [alpha_sindy,beta_sindy,vm_sindy,s0_sindy,s_star_sindy];

batch_error_sindy = ARED_rmse(p_sindy,accel_func,veh_T,veh_S,veh_V,veh_VL,'spacing');

%% Perform a sim:

[t,veh_X_sim,veh_V_sim,veh_S_sim] = sim_ring(p_sindy,accel_func,21,153.91,400,1/30,true);

figure()
subplot(2,1,1)
plot(t,veh_V_sim')
ylabel('Speed [m/s]')
title('SINDy parameters')
subplot(2,1,2)
plot(t,veh_S_sim')
ylabel('Spacing [m]')


[t,veh_X_sim,veh_V_sim,veh_S_sim] = sim_ring(p_batch,accel_func,21,153.91,400,1/30,true);
figure()
subplot(2,1,1)
plot(t,veh_V_sim')
ylabel('Speed [m/s]')
title('Batch parameters')
subplot(2,1,2)
plot(t,veh_S_sim')
ylabel('Speed [m/s]')

























