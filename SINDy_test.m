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
t = follower_traj(:,1);
v = follower_traj(:,2);
v_l = leader_traj(:,2);
s = follower_traj(:,4);
delta_v = v_l - v;
%% Load library functions:
% Bando_Accel = @(p,s,ds,v)  alpha*(vm*(tanh(s./d0-2)+tanh(2))/(1+tanh(2))-v) + beta*((ds)./(s.^2));

% Bando_Accel = @(p,s,ds,v) alpha*(vm*(tanh(s./d0-2)+tanh(2))/(1+tanh(2))-v) + beta*((ds)./(s.^2));


d0 = 2.23; % reference distance [m]
% vm = 9.72; % maximum velocity [m/s]

V = @(s) (tanh(s./d0-2)+tanh(2))/(1+tanh(2)); % optimal velocity function

% alpha*vm*V(s) - alpha*v -> prod([V(s),v],[alpha*vm,-alpha])

DV = @(ds_dt,s) (ds_dt./(s.^2));

% beta*DV(ds_dt,s) -> prod([DV(ds_dt,s)],[beta])

% alpha*vm*V(s) - alpha*v + beta*DV(ds_dt,s) =
% prod([V(s),v,DV(ds_dt,s)],[alpha*vm,-alpha,beta]


% IDM: @(p,s,ds,v) a*(1 - (v/v0)^delta -(s_star(ds_dt,s)/s)^2)

% s_


%% Load matrix and regress on:

dt = diff(t);


D = [V(s(1:end-1)).*dt,...
    -v(1:end-1).*dt,...
    DV(delta_v(1:end-1),s(1:end-1)).*dt];

y = v(1:end-1) - v(2:end);

p = pinv(D)*y;

%% extract parameters and 

alpha = p(2);
beta = (3);
vm = p(1)/alpha;


%% run a sim using params:

accel = @(v,s,ds_dt) alpha*(vm*V(s)-v) + beta*DV(ds_dt,s);























