function [z] = sensormodel(b_position, state_t, sigma)
    % b_position: beacon position [x, y]
    % state_t: robot position at time t [x, y]
    % sigma: distance and angle measure uncertainty [sigma_dist, sigma_angl]
    %
    % based from https://github.com/UTS-CAS/Robot-Localization-examples

    b_x = b_position(1);
    b_y = b_position(2);

    x = state_t(1);
    y = state_t(2);

    sigma_dist = sigma(1);
    sigma_angl = sigma(2);

    m_dist = norm([b_x, b_y] - [x, y]) + sigma_dist;
    m_angl = atan2((b_y - y), (b_x - x)) + sigma_angl;

    z = [m_dist, m_angl];

end