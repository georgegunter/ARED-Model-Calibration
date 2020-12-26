function [error] = rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real)
    [~,s_sim] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real);
    error = sqrt(mean((s_real-s_sim).^2));
end