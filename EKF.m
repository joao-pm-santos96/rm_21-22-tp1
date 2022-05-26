function [state_t1, P_t1] = EKF(state_t, P_t, control_t, obs_t1, landmarks, delta_t, Q, R, sigma_motion, sigma_sensor)
    % state: pose of robot: [x ,y, alpha]
    % P_t:
    % control_t: control input at time t [lin_v, ang_v]
    % obs_t1: measured distance and angle to landmarks Nx2: [dist, angl]
    % landmakrs: landmarks positions Nx2: [x, y]
    % delta_t: time interval
    % Q:
    % R:
    % sigma_motion: 
    % sigma_sensor: 
    %
    % based from https://github.com/UTS-CAS/Robot-Localization-examples

    %% Prediction

    state_t1_temp = MotionModel(state_t, control_t, sigma_motion, delta_t);
    
    % TODO which is right?
%     j1 = - delta_t * control_t(1) * sin(state_t(3));
%     j2 = - delta_t * control_t(1) * cos(state_t(3));
%     j3 = - j2;
%     j4 = - j1;
% 
%     Jfx = [1 0 j1
%             0 1 j2
%             0 0 1];
% 
%     Jfw = [j3 0
%             j4 0
%             0 delta_t];

    theta_k = state_t(3);
    v_k = control_t(1);
    Jfx = [1 0 -delta_t.*v_k.*sin(theta_k); 0 1 delta_t.*v_k.*cos(theta_k); 0 0 1];
    Jfw = [delta_t.*cos(theta_k) 0; delta_t.*sin(theta_k) 0; 0 delta_t];

    P_t1_temp = Jfx * P_t * Jfx' + Jfw * Q * Jfw';

    %% Update
    N = size(landmarks,1);
    z_all = reshape(obs_t1',[],1);
    z_pred = [];

    for n=1:1:N
        z = SensorModel(landmarks(n,:), state_t1_temp, sigma_sensor);
        z(1,2) = wrapToPi(z(1,2));
        z_pred = [z_pred; z'];
    end
    innov = z_all - z_pred;

    Jh = [];
    for n=1:1:N
        x_k1 = state_t1_temp(1);
        y_k1 = state_t1_temp(2);

        x_l = landmarks(n,1);
        y_l = landmarks(n,2);

        % TODO which is right?
        J = [(x_k1 - x_l)./sqrt((-x_k1 + x_l).^2 + (-y_k1 + y_l).^2) (y_k1 - y_l)./sqrt((-x_k1 + x_l).^2 + (-y_k1 + y_l).^2) 0
            (-y_k1 + y_l)./((1 + (-y_k1 + y_l).^2./(-x_k1 + x_l).^2).*(-x_k1 + x_l).^2) -1./((1 + (-y_k1 + y_l).^2./(-x_k1 + x_l).^2).*(-x_k1 + x_l)) -1];
        % Jh = [Jh; Jacobi(landmarks(n,:), state_t1_temp)];
        Jh = [Jh;J];
    end
    
    S = Jh * P_t1_temp * Jh' + R;
    K = P_t1_temp * Jh' / S;

    %% Result
    xstatet1_t1 = state_t1_temp' + K * innov;
    Pt1_t1 = P_t1_temp - K * Jh * P_t1_temp;
    
    state_t1 = xstatet1_t1';
    P_t1 = Pt1_t1;

end




