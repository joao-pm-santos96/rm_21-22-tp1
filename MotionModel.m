function pose = MotionModel(x_t, u_t, w_t, dt)
    
    T = [cos(x_t(3)) 0
        sin(x_t(3)) 0
        0 1];

    pose = ((T * (u_t + w_t)') .* dt + x_t')';

end