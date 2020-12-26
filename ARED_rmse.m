function [mean_rmse] = ARED_rmse(p,accel_func,veh_T,veh_S,veh_V,veh_VL,error_string)

num_vehicles = length(veh_S(:,1));

mean_rmse = 0;

if(strcmp(error_string,'spacing'))
    for i=1:num_vehicles
        t_real = veh_T(i,:);
        s_real = veh_S(i,:);
        v_real = veh_V(i,:);
        v_l_real = veh_VL(i,:);
        
        rmse = rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real);
        
        mean_rmse = mean_rmse + rmse;
         
    end
else
    for i=1:num_vehicles
        t_real = veh_T(i,:);
        s_real = veh_S(i,:);
        v_real = veh_V(i,:);
        v_l_real = veh_VL(i,:);

        rmse = rmse_speed(p,accel_func,t_real,s_real,v_real,v_l_real);

        mean_rmse = mean_rmse + rmse;       
    end
end

mean_rmse = mean_rmse/num_vehicles;
             
end

