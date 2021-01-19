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
%% Select a trajectory from one of the drivers:
veh_index = 1;
t_real = veh_T(veh_index,:);
s_real = veh_S(veh_index,:);
v_real = veh_V(veh_index,:);
v_l_real = veh_VL(veh_index,:);

%% Choose a set of params for simulation:
V = @(p,s) p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)));
accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));
p_real = [.8,10,11.0,2.23,5]; %[alpha,beta,Vm,d0,d_star]

[v_sim,s_sim] = ...
    run_sim(p_real,accel_func,t_real,s_real,v_real,v_l_real);
ds_dt_sim = v_l_real - v_sim;

accel_real = accel_func(p_real,s_sim,ds_dt_sim,v_sim);


%% Add noise to the velocity:

sigma = 0.5;
noise = randn(1,length(v_sim))*sigma;
v_sim_noise  = v_sim + noise;

%% Make accel estimate:
dt = 1/30;
dv_dt = gradient(v_sim_noise,dt);

figure()
plot(accel_real-dv_dt,'LineWidth',1)
ylabel('Accel error [m/s^2]')
set(gca,'FontSize',24)

%% Make basis functions for regression:
s0_sindy = p_real(4);
s_star_sindy = p_real(5);

V_sindy =  @(s) (tanh(s./s0_sindy-s_star_sindy)+tanh(s_star_sindy))/(1+tanh(s_star_sindy));  

FTL_sindy = @(s,ds_dt) (ds_dt./(s.^2));
%% Load matrix to regress on:

A = [V_sindy(s_sim)',v_sim',FTL_sindy(s_sim,ds_dt_sim)'];

%% Perform regression:

tic
x_sindy = pinv(A)*dv_dt';
toc
disp('Regression finished')

alpha_sindy = -x_sindy(2);
beta_sindy = x_sindy(3);
vm_sindy = x_sindy(1)/alpha_sindy;
p_sindy = [alpha_sindy,beta_sindy,vm_sindy,s0_sindy,s_star_sindy];

%% Look at error from simulation:

[v_sindy,s_sindy] = ...
    run_sim(p_sindy,accel_func,t_real,s_real,v_real,v_l_real);

figure()
subplot(2,1,1)
hold on
plot(t_real,v_sim-v_sindy)
ylabel('Speed error [m/s]')
set(gca,'FontSize',24)
subplot(2,1,2)
hold on
plot(t_real,s_sim-s_sindy)
ylabel('Spacing error [m]')
xlabel('Time [s]')
set(gca,'FontSize',24)










