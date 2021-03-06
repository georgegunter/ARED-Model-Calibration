function [p_opt,error_opt] = ...
    batch_calibrate_single(t_real,...
    v_real,...
    v_l_real,...
    s_real,...
    accel_func,...
    p0)
%% calibration:

    obj_func = @(p) rmse_spacing(p,accel_func,t_real,s_real,v_real,v_l_real);
    options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
    [p_opt,error_opt] = fminunc(obj_func,p0,options);
    
end

