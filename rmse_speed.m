function [error] = rmse_speed(p,t_real,s_real,v_real,v_l_real)
    [v_sim,~] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real);
    error = sqrt(mean((v_real-v_sim).^2));
end