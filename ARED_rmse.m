function [mean_rmse] = ARED_rmse(p,accel_func,veh_T,veh_X,veh_S,veh_V,veh_VL,error_string)

num_vehicles = length(veh_S(:,1));

mean_rmse = 0;

for i=1:num_vehicles
    t_real = veh_T(i,:);
    s_real = veh_S(i,:);
    v_real = veh_V(i,:);
    v_l_real = veh_VL(i,:);
    p_real = veh_X(i,:)-veh_X(i,1); %Make positions start from 0

    rmse = get_rmse(p,accel_func,t_real,s_real,p_real,v_real,v_l_real,error_string);

    mean_rmse = mean_rmse + rmse;
    
end
         
mean_rmse = mean_rmse/num_vehicles;
             
end

