function [v_sim,s_sim,a_sim] = run_sim(p,accel_func,t_real,s_real,v_real,v_l_real)
    v_sim = zeros(size(v_real));
    s_sim = zeros(size(s_real));
    
    v_sim(1) = v_real(1);
    s_sim(1) = s_real(1);
    
    num_steps = length(t_real);
    
    for i=2:num_steps
        
        v = v_sim(i-1);
        s = s_sim(i-1);
        v_l = v_l_real(i-1);
        ds_dt = v_l - v;
        dv_dt = accel_func(p,s,ds_dt,v);
        dt = t_real(i) - t_real(i-1);
        
        v_sim(i) = v + dt*dv_dt;
        s_sim(i) = s + dt*ds_dt;
        
    end
end