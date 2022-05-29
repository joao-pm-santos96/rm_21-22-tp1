function z = SensorModel(state_t, lm_xy, sigma)
% SensorModel Sensor model for LiDAR
%   z = SensorModel(state_t, lm_xy, sigma)
%
%   state_t: robot position at time t [x, y]
%   lm_xy: landmark position [x, y]
%   sigma: distance and angle measure uncertainty [sigma_dist, sigma_angl]
    
    delta = lm_xy - state_t(1:2);

    r = norm(delta);
    phi = atan2(delta(2), delta(1)) - state_t(3);

    z = [r phi] + sigma;

end