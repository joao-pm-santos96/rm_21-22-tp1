function z = SensorModel(x_t, lm_xy, v_t)
    
    delta = lm_xy - x_t(1:2);

    r = norm(delta);
    phi = atan2(delta(2), delta(1)) - x_t(3);

    z = [r phi] + v_t;

end