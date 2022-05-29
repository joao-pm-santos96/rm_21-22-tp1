function state_t1 = MotionModel(state_t, input_t, sigma, dt)
% MotionModel Motion model for the general mobile robot in 2D
%   state_t1 = MotionModel(prev_state, c_in, sigma, dt)
%   
%   prev_state: previous state [x, y, alpha]
%   c_in: control input [v_lin, v_ang]
%   sigma: linear and angular velocity uncertainty [sigma_lin, sigma_ang]
%   dt: time interval
    
    T = [cos(state_t(3)) 0
        sin(state_t(3)) 0
        0 1];

    state_t1 = ((T * (input_t + sigma)') .* dt + state_t')';

end