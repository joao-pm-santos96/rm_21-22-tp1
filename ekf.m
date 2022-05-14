function [state_t1, P_t1] = ekf(state_t, P_t, control_t, obs_t1, landmarks, delta_t, Q, R)
    % 
    % obs_t1: measured distance and angle to landmarks Nx2: [dist, angl]
    % landmakrs: landmarks positions Nx2: [x, y]
    %
    % based from https://github.com/UTS-CAS/Robot-Localization-examples

    %% Prediction

    sigma_1 = [0,0]; %TODO shouldn't be...
    state_t1 = motionmodel(state_t, control_t, sigma_1, delta_t);
    
    j1 = - delta_t * control_t(1) * sin(state_t(3));
    j2 = - delta_t * control_t(1) * cos(state_t(3));
    j3 = - j2;
    j4 = - j1;

    Jfx = [1 0 j1
            0 1 j2
            0 0 1];

    Jfw = [j3 0
            j4 0
            0 delta_t];

    P_t1_temp = Jfx * P_t * Jfx' + Jfw * Q * Jfw';

    %% Update
    N = size(landmarks,1);
    z_all = obs_t1;
    z_pred = zeros(N,2);

    % TODO check that the order is always maintained
    % TODO watch out for NANs...
    sigma_2 = [0,0]; %TODO shouldn't be...
    for n=1:1:N
        z_pred(n,:) = sensormodel(landmarks(n,:), state_t, sigma_2);
    end
    innov = z_all - z_pred;








end