function [next_state] = motionmodel(prev_state, c_in, sigma, delta_t)
    % prev_state: previous state [x, y, alpha]
    % c_in: control input [v_lin, v_ang]
    % sigma: linear and angular velocity uncertainty [sigma_lin, sigma_ang]
    %
    % based from https://github.com/UTS-CAS/Robot-Localization-examples

    x = prev_state(1);
    y = prev_state(2);
    a = prev_state(3);

    sigma_lin = sigma(1);
    sigma_ang = sigma(2);

    v_lin = c_in(1);
    v_ang = c_in(2);

    new_x = x + ((v_lin + sigma_lin) * delta_t * cos(a));
    new_y = y + ((v_lin + sigma_lin) * delta_t * sin(a));
    new_a = a + ((v_ang + sigma_ang) * delta_t);

    next_state = [new_x, new_y, new_a];

end