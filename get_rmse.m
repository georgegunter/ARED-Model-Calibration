function [error] = get_rmse(p,accel_func,t_real,s_real,p_real,v_real,v_l_real,error_string)
    [v_sim,s_sim,p_sim] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real);
    
    if(strcmp(error_string,'speed'))
        error = sqrt(mean((v_real-v_sim).^2));
    elseif(strcmp(error_string,'spacing'))
        error = sqrt(mean((s_real-s_sim).^2));
    elseif(strcmp(error_string,'position'))
        error = sqrt(mean((p_real-p_sim).^2));
    end
        
end