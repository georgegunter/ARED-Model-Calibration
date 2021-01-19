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

%Default values are picked so that can converge directly to what is found
%through batch calibration:
s0_sindy = 2.21;
s_star_sindy = 2.815;

V_sindy =  @(s) (tanh(s./s0_sindy-s_star_sindy)+tanh(s_star_sindy))/(1+tanh(s_star_sindy));  
FTL_sindy = @(s,ds_dt) (ds_dt./(s.^2));
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

A = [V_sindy(s_vec),v_vec,FTL_sindy(s_vec,dsdt_vec)];

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
subplot(2,1,2)
plot(t,veh_S_sim')
ylabel('Spacing [m]')

























