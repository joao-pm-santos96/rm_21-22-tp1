function [x_t1, P_t1] = MyEKF2(x_t, u_t, z_t1, P_t, lm_xy, Q, R, dt)
% x_t: current pose estimate
% u_t: current control input
% z_k1: observations after moving to next pose N x [dist ang]
% P_t: current variance/covariance
% lm_xy: landmark positions [x y]
% Q:
% R:
    
    w_t = [0,0];
    v_t = [0,0];
    
    %% Prediction
    jfx = Jfx(x_t, u_t, w_t, dt);
    jfw = Jfw(x_t, dt);
    
    x_t1_pred = MM(x_t, u_t, w_t, dt);
    P_t1_pred = jfx * P_t * jfx' + jfw * Q * jfw';       
    
    %% Update
    jh = [];
    for n=1:1:size(lm_xy,1)
        jh = [jh; Jh(x_t1_pred, lm_xy(n,:))];
    end
    

    S = jh * P_t1_pred * jh' + R; % Innovation covariance
    K = P_t1_pred * jh' * inv(S); % Kalmn gain

    z_pred = [];
    for n=1:1:size(lm_xy,1)       
        z = SS(x_t1_pred, lm_xy(n,:), v_t);
        z_pred = [z_pred; z];
    end

    innov = z_t1 - z_pred; % Innovation
    innov = reshape(innov',[],1);

    x_t1 = (x_t1_pred' + K * innov)';
%     S
jfx
jfw
Q
P_t1_pred

S
%     P_t1 = P_t1_pred + K * S * K';
%     P_t1 = P_t1';
%     P_t1 = (eye(3) - K * jh)*P_t1_pred;


    %% DEBUG

    

%     rr = z_t1(:,1);
%     angg = z_t1(:,2) + x_t1_pred(3);
%     XX = ones(size(z_t1,1),1).*x_t1_pred(1);
%     YY = ones(size(z_t1,1),1).*x_t1_pred(2);
%     Xa = rr.*cos(angg);
%     Ya = rr.*sin(angg);       
%     quiver(XX, YY, Xa, Ya, 'off', 'c-')
%     hold on
% 
%     rr = z_pred(:,1);
%     angg = z_pred(:,2) + x_t1_pred(3);
%     XX = ones(size(z_pred,1),1).*x_t1_pred(1);
%     YY = ones(size(z_pred,1),1).*x_t1_pred(2);
%     Xa = rr.*cos(angg);
%     Ya = rr.*sin(angg);
%     quiver(XX, YY, Xa, Ya, 'off', 'm--')
%     hold on

    plot(lm_xy(:,1), lm_xy(:,2), 'rd')
    hold on

    plot(x_t1_pred(1), x_t1_pred(2), 'b*')
    hold on

    plot(x_t1(1), x_t1(2), 'ks')
    hold on




    legend('beacons','path','z\_t1','z\_pred','lm\_xy', 'x\_t1\_pred','x\_t1')


end















function pose = MM(x_t, u_t, w_t, dt)
    
    theta = x_t(3);

    T = [cos(theta) 0
        sin(theta) 0
        0 1];

    pose = ((T * (u_t' + w_t')) * dt + x_t')';

end

function h = SS(x_t, lm_xy, v_t)

    delta = lm_xy - x_t(1:2);

    r = norm(delta);
    phi = atan2(delta(2), delta(1)) - x_t(3);

    h = [r phi] + v_t;

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




