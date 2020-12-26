%% Initialize path:

% Past the path to ARED data here:
ARED_path  = '/Users/vanderbilt/Desktop/Research_2020/ARED/Cleaned Trajectory Data/';

follower_traj = readmatrix(strcat(ARED_path,'Vehicle_1_Experiment_A.csv'));
leader_traj = readmatrix(strcat(ARED_path,'Vehicle_2_Experiment_A.csv'));

begin_point = 1650;
end_point = 11550;

follower_traj = follower_traj(begin_point:end_point,:);
leader_traj = leader_traj(begin_point:end_point,:);
%% load trajectory:
t_real = follower_traj(:,1);
v_real = follower_traj(:,2);
v_l_real = leader_traj(:,2);
s_real = follower_traj(:,4);

%% Look at calibrating OV-FTL:
d0 = 2.23; %Leave out this variable, would normally be included...

accel_func = @(p,s,ds_dt,v)p(1)*(p(2)*(tanh(s./d0-2)+tanh(2))/(1+tanh(2))-v)+p(3)*((ds_dt)./(s.^2));

obj_func = @(p) rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real);
p0 = [.3,9.7,20];

%% Run optimization:
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
[p_opt,error_opt] = fminunc(obj_func,p0,options);


%% Look at calibrating IDM:

accel_func = @(p,s,ds,v) p(1).*( 1 - (v/p(3)).^p(4) - ...
    ((p(6) + max(p(5)*v + v.*(-ds)./(2*sqrt(p(1).*p(2))),0))./s).^2 );
p0 = [1.3,2.0,12.0,4.0,1.2,2.0];
obj_func = @(p) rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real);

%% Run optimization:
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
[p_opt_IDM,error_opt_IDM] = fminunc(obj_func,p0,options);

%% Simulation functions:

function [error] = rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real)
    [~,s_sim] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real);
    error = sqrt(mean((s_real-s_sim).^2));
end

function [error] = rmse_speed(p,t_real,s_real,v_real,v_l_real)
    [v_sim,~] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real);
    error = sqrt(mean((v_real-v_sim).^2));
end

