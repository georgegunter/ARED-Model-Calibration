function [MRho,MQ,jam_speed,mt,mx] = ...
    get_macro_filter_data(times,ring_length,veh_n,veh_X,veh_V)
%% Extract macroscopic flow property fields
tf = times(end);
dt = times(2) - times(1);

veh_X_circ = mod(veh_X,ring_length);

wx = 20; % width of kernel in x
wt = 10; % width of kernel in t

mt = 0:5:tf;
mx = linspace(0,ring_length,floor(ring_length));

[MT,MX] = meshgrid(mt,mx);

MRho = MT*0;% Mesh for densities
MQ = MT*0;% Mesh for flows

% Gaussian_kernel = @(wx,wt) exp(-(dist/wx).^2-...
%     ((MT-t(i))/wt).^2)/(pi*wx*wt)*dt;% Gaussian kernel 
disp('Running macro filter...')
%% 
for j = 1:veh_n % loop over all vehicles trajectories
    if floor(j/veh_n*50)>floor((j-1)/veh_n*50) % progress marker
        fprintf('^')
    end
    for i = 1:length(times) % loop over all points in that trajectory
        
        dist = min(abs(MX-veh_X_circ(j,i)),abs(MX-veh_X_circ(j,i)-ring_length));
        dist = min(dist,abs(MX-veh_X_circ(j,i)+ring_length));
        % Apply a guassian kernel to smooth:
        G = exp(-(dist/wx).^2-...
            ((MT-times(i))/wt).^2)/(pi*wx*wt)*dt; % Gaussian kernel
        
        MRho = MRho+G; % update vehicle density field
        MQ = MQ+G*veh_V(j,i); % update flow rate field
    end
end
fprintf('\n')
disp('finished with kernel filtering.')
%% LSTSQ Rrgression:
fprintf('\n')

for i=1:length(mt)
%     [Max_rho(i), indexx] = max(MRho(:,i));
%     pos_max(i) = mx(indexx);
    jam_line(i,:) = polyfit(1000*MRho(:,i),3600*MQ(:,i),1);
    line_ordinates(i,:) = polyval(jam_line(i,:),[0,140]);
    jam_speed(i) = jam_line(i,1);
end
disp('Jam line found.')