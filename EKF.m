function [x_t1, P_t1] = EKF(x_t, u_t, z_t1, P_t, lm_xy, Q, R, dt)
% x_t: current pose estimate [x y theta]'
% u_t: current control input [v w]'
% z_k1: observations after moving to next pose N x [dist ang]
% P_t: current variance/covariance
% lm_xy: landmark positions [x y]
% Q: covariance matrix for the process noise
% R:covariance matrix for the observation noise
    
    %% Defaults
    w_t = [0 0];
    v_t = [0 0];

    %% Prediction    
    jfx = Jfx(x_t, u_t, w_t, dt);
    jfw = Jfw(x_t, dt);

    x_t1_pred = MotionModel(x_t, u_t, w_t, dt);
    P_t1_pred = jfx * P_t * jfx' + jfw * Q * jfw';  

    %% Update
    jh = [];
    for n=1:1:size(lm_xy,1)
        jh = [jh; Jh(x_t1_pred, lm_xy(n,:))];
    end

    z_all = reshape(z_t1', [], 1);
    z_pred = [];
    for n=1:1:size(lm_xy,1)
        z_pred = [z_pred; SensorModel(x_t1_pred, lm_xy(n,:), v_t)'];
    end
    innov = z_all - z_pred;

    S = jh * P_t1_pred * jh' + R;
    K = P_t1_pred * jh' * pinv(S);

    x_t1 = (x_t1_pred' + K * innov)';
    P_t1 = P_t1_pred - K * S * K';

end






function jacob = Jfx(x_t, u_t, w_t, t)

    jacob = [1 0 -t.*(w_t(1) + u_t(1)).*sin(x_t(3))
        0 1 t.*(w_t(1) + u_t(1)).*cos(x_t(3))
        0 0 1];
end

function jacob = Jfw(x_t, t)
    
    jacob = [t.*cos(x_t(3)) 0
        t.*sin(x_t(3)) 0
        0 t];
end

function jacob = Jh(x_t1_pred, lm_xy)

    delta = x_t1_pred(1:2) - lm_xy;

    a = (delta(1))./sqrt((-delta(1)).^2 + (-delta(2)).^2);
    b = (delta(2))./sqrt((-delta(1)).^2 + (-delta(2)).^2);
    c = 0;
    d = (-delta(2))./((1 + (-delta(2)).^2./(-delta(1)).^2).*(-delta(1)).^2);
    e = -1./((1 + (-delta(2)).^2./(-delta(1)).^2).*(-delta(1)));
    f = -1;
    
    jacob = [a b c
        d e f];

end