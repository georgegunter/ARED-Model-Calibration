load_data = false;
if(load_data)    
%% Load data:
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
end
%% simulation parameters:

accel_func = @(p,s,ds,v)  p(1)*(p(3)*(tanh(s./p(4)-p(5))+tanh(p(5)))/(1+tanh(p(5)))-v) + p(2)*((ds)./(s.^2));
% p = [0.6660,21.5975,8.9368,2.2146,2.8150]; %Batch calibration
p = [0.0947,8.1310,10.1689,2.2100,2.8150]; %SINDY calibration

veh_n = 21;
ring_length = 153.9;
sim_length = veh_T(1,end) - veh_T(1,1);
dt=1/30;
print_progress = true;

%% Simulate a ring initialized according to ARED:

noise = 3e-1; % magnitude of velocity noise in each step
noise0 = 1e-1*0; % magnitude of velocity noise initially
t_noise = 100; % time after which noise is stopped
integrator = 'ballistic';
%integrator = 'Euler';

% Simulation parameters
tf = sim_length; % final time
L = ring_length; % road length

acceleration = @(s,ds,v) accel_func(p,s,ds,v);

% Explanation: Input parameters are:
% s = gap to lead vehicle (x_lead-x_self-length)
% ds = velocity difference from lead vehicle
%      (v_lead-v_self) = -approaching rate [note the difference to IDM]
% v = vehicle velocity
% Output is vehicle acceleration

%% Simulation initialization
%rng(0)
n_steps = ceil(tf/dt); dt = tf/n_steps;
%s_eq = S_eq(v_eq); % corresponding equilibrium gap
%d_eq = s_eq+len; % equilibrium spacing
d_eq = L/veh_n; % road length
s_eq = d_eq;

% Search for velocity eq:

fun = @(v) acceleration(s_eq,0,v);
v_eq = fzero(fun,[0+1e-4,30]); %find equilibrium speed
%% Initialize state variables:
veh_v = veh_V(:,1);                  % initial vehicle velocities
veh_s = veh_S(:,1);
veh_x = cumsum(veh_s);        % initial vehicle positions
veh_X_sim = veh_X*0; veh_X_sim(:,1)=veh_x; % array storing time-history of positions
veh_V_sim = veh_V*0; veh_V_sim(:,1)=veh_v;     % array storing time-history of velocities
veh_S_sim = veh_S*0; veh_S_sim(:,1)=veh_s;  % array storing time-history of gaps

%% Time loop
t = linspace(0,tf,n_steps+1); % time vector
if(print_progress)
    fprintf('_______20%%_______40%%_______60%%_______80%%______100%%\n')
end
for j = 2:n_steps
    if(print_progress)
        if floor(j/n_steps*50)>floor((j-1)/n_steps*50) % progress marker
            fprintf('^')
        end
    end
    % Update step
    veh_s = [veh_x(2:end);veh_x(1)+L]-veh_x;         % vehicle gaps
    veh_ds = veh_v([2:end,1])-veh_v;                     % velocity differences
    dx = veh_v; dv = acceleration(veh_s,veh_ds,veh_v);   % rates of change
    if strcmp(integrator,'ballistic')
        veh_x = veh_x+max(dt*dx+dt^2/2*dv,0);
        veh_v = veh_v+dt*dv;                             % ODE update (ballistic)
    else
        veh_x = veh_x+dt*dx; veh_v = veh_v+dt*dv;        % ODE update (Euler)
    end
    
    veh_v = max(veh_v,0);                                % cap vehicle velocity from below
    
    veh_X_sim(:,j+1) = veh_x;                               % save current state to array
    veh_V_sim(:,j+1) = veh_v;                               % save current state to array
    veh_S_sim(:,j+1) = veh_s;                               % save current state to array
end
fprintf('\n')

%% Plotting:
figure()
subplot(2,1,1)
scatter(reshape(veh_T,numel(veh_T),1),mod(reshape(veh_X_sim,numel(veh_X_sim),1),153.9),1,reshape(veh_V_sim,numel(veh_V_sim),1))
xlim([min(veh_T,[],'all'),max(veh_T,[],'all')])
title('Optimal Model space-time')
set(gca,'FontSize',20)
set(gca,'LineWidth',3)
ylabel('Postion [m]')

subplot(2,1,2)
scatter(reshape(veh_T,numel(veh_T),1),mod(reshape(veh_X,numel(veh_X),1),153.9),1,reshape(veh_V,numel(veh_V),1))
xlim([min(veh_T,[],'all'),max(veh_T,[],'all')])
title('ARED experiment space-time')
set(gca,'FontSize',20)
set(gca,'LineWidth',3)
ylabel('Postion [m]')
xlabel('Time [s]')