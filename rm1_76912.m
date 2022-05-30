function rm1_76912(N, Dt, r, L, Vn, Wn, V)
% RM1_76912 TP1 de Robotica Movel | 2021-22
%   João Santos, 76912
%
%   N: número de faróis (default: 4)
%   Dt: intervalo de tempo de amostragem dos sensores (default: 1s)
%   r: raio das rodas dos robots (default: 0.15m)
%   L: separação/afastamento das rodas conforme o modelo cinemático (default: 1m)
%   Vn: incerteza (sigma) na velocidade linear a impor ao robot (default: 0.1m/s)
%   Vw: incerteza (sigma) na velocidade angular a impor ao robot (default: 0.1rad/s)
%   V: velocidade média desejada (default: 5m/s)

    %% Default arguments
    arguments
        N = 4 
        Dt = 1 
        r = 0.15 
        L = 1
        Vn = 0.1
        Wn = 0.1
        V = 5
    end

    %% Constants 
    INITIAL_POSE = [0,0,0];
    LOC_FILE = 'loc_76912.txt';
    DD_FILE = 'DD_76912.txt';
    TRI_FILE = 'TRI_76912.txt';
    OMNI_FILE = 'OMNI_76912.txt';
    DEBUG = true; % TODO set to false!

    if (DEBUG)
        close all
        clc
    end
    
    %% Step 1: Compute all path points
    if (DEBUG)
        disp('Step 1')
    end

    beacons = BeaconDetection(N);

    % Get all known points
    beacon_poses = [beacons(:).X; beacons(:).Y; beacons(:).a]';
    known_poses = [INITIAL_POSE; beacon_poses];

    % Get intermediate points per section
    all_poses = INITIAL_POSE;
    for n=1:1:N
        % Compute distance between two points
        p0 = known_poses(n,:);
        p1 = known_poses(n+1,:);
        d = norm(p1-p0);
        
        % Get n points
        n_points = round(d / (V * Dt));

        deltas = (1:1:n_points)/n_points;
        step_poses = p0+deltas'*(p1-p0);
        all_poses = [all_poses; step_poses];
    end

    % Get smooth path coordinates
    yq = pchip(known_poses(:,1), known_poses(:,2), all_poses(:,1));
    smooth_path = [all_poses(:,1), yq];

    % Get smooth path orientations
    orients = INITIAL_POSE(3); 
    for n=2:1:size(smooth_path,1)
        p0 = smooth_path(n-1,:);
        p1 = smooth_path(n,:);
        v = p1 - p0;
        orients = [orients; atan2(v(2),v(1))];
    end
    smooth_path(:,3) = orients;
    
    %%% DEGUB %%%
    if (DEBUG)
        figure
        plot(known_poses(:,1), known_poses(:,2),'bo')
        hold on
        plot(known_poses(:,1), known_poses(:,2),'r--')
        hold on
        plot(smooth_path(:,1),smooth_path(:,2),'g-')
        grid on
        title('Path')
    end
    %%%%%%%%%%%%%

    %% Step 2: Compute velocities
    if (DEBUG)
        disp('Step 2')
    end

    control_inputs = []; 
    for n=1:1:size(smooth_path,1)-1
        p0 = smooth_path(n,:);
        p1 = smooth_path(n+1,:);
        v = p1 - p0;
        lin_vel = norm(v(1:2)) / Dt;
        ang_vel = v(3) / Dt;
        control_inputs = [control_inputs; [lin_vel, ang_vel]];
    end
    control_inputs(end+1,:) = [0, 0];

    % Add RAND noises
    % noises_v = randn(size(control_inputs,1),1) .* Vn;
    % noises_w = randn(size(control_inputs,1),1) .* Wn;
    noises_v = normrnd(0, Vn);
    noises_w = normrnd(0, Vw);
    control_inputs(:,1) = control_inputs(:,1) + Vn;
    control_inputs(:,2) = control_inputs(:,2) + Wn;

    %%% DEBUG %%%
    if (DEBUG)
        figure
        plot(smooth_path(:,1),smooth_path(:,2),'g-')
        hold on
        vel = control_inputs(:,1);
        quiver(smooth_path(:,1), smooth_path(:,2), vel.*cos(smooth_path(:,3)), vel.*sin(smooth_path(:,3)), 'off')
        grid on
        title('Velocity vectors')
    end
    %%%%%%%%%%%%%

    %% Step 3: Extended Kalman Filter 
    if (DEBUG)
        disp('Step 3')
    end

    % Initial values
    state_t = INITIAL_POSE;
    P_t = eye(3) * 0.01;
    Q = [Vn^2 0
        0 Wn^2];

    ekf_loc = state_t;
    ekf_p = P_t;

    % TODO add try catch
    for n=1:1:size(control_inputs,1)

        control_t = control_inputs(n,:);
        state_t1 = MotionModel(state_t, control_t, [0 0], Dt);
        
        B = BeaconDetection(N, state_t1);
        
        obs_t1 = [B(:).d; B(:).a]';
        lm_xy = [B(:).X; B(:).Y]';
        obs_n = [B(:).dn; B(:).an]';
        
        % Deal with NaNs
        [rows, ~] = find(isnan(obs_t1));
        nan_rows = unique(rows);

        obs_t1(nan_rows, :) = [];
        lm_xy(nan_rows, :) = [];
        obs_n(nan_rows, :) = [];

        % Deal with collinear landmarks-robot
        threshold = 0.1;
        [rows, ~] = find(obs_t1(:,2) > -threshold && obs_t1(:,2) < threshold);
        coll_rows = unique(rows);

        obs_t1(coll_rows, :) = [];
        lm_xy(coll_rows, :) = [];
        obs_n(coll_rows, :) = [];

        b_noises = reshape(obs_n',1,[]);
        R = diag(b_noises).^2;

        [state_t1, P_t1] = EKF(state_t, control_t, obs_t1, P_t, lm_xy, Q, R, Dt);

        state_t = state_t1;
        P_t = P_t1;

        ekf_loc = [ekf_loc; state_t];
        ekf_p = [ekf_p; P_t];       

    end

    if (DEBUG)
        figure
        plot(smooth_path(:,1),smooth_path(:,2),'g-')
        hold on        
        tmp = 2;
        quiver(ekf_loc(:,1), ekf_loc(:,2), tmp*cos(ekf_loc(:,3)), tmp*sin(ekf_loc(:,3)), 'off')
        title('EKF')
        grid on
    end   

    %% Step 4: Write localization file
    if (DEBUG)
        disp('Step 4')
    end
    writematrix(ekf_loc, LOC_FILE);

    %% Step 5: Kinematic Models
    if (DEBUG)
        disp('Step 5')
    end

    % Differential drive
    diff_wheels = [];
    for n=1:1:size(smooth_path,1)-1
        wheels = DiffDriveIK(r, L, smooth_path(n,:), smooth_path(n+1,:), Dt); 
        diff_wheels = [diff_wheels; [wheels.wr wheels.wl]]; 
    end
    diff_wheels(end+1,:) = zeros(1, size(diff_wheels, 2)); % TODO should end in vles=0?
    writematrix(diff_wheels, DD_FILE);

    if (DEBUG)
        figure
        plot(smooth_path(:,1), diff_wheels(:,1)) 
        hold on
        plot(smooth_path(:,1), diff_wheels(:,2))
        grid on
        legend('wr', 'wl')
        title('Differential Drive')
    end
    
    % Tricyle
    tri_wheels = [];
    for n=1:1:size(smooth_path,1)-1
        wheels = TricycleIK(r, L, smooth_path(n,:), smooth_path(n+1,:), Dt); 
        tri_wheels = [tri_wheels; [wheels.wt wheels.alpha]];     
    end
    tri_wheels(end+1,:) = zeros(1, size(tri_wheels, 2)); % TODO should end in vles=0?
    writematrix(tri_wheels, TRI_FILE);

    if (DEBUG)
        figure
        plot(smooth_path(:,1), tri_wheels(:,1)) 
        hold on
        yyaxis right
        plot(smooth_path(:,1), tri_wheels(:,2))
        grid on
        legend('wt', 'alpha')
        title('Tricycle')
    end

    % Omnidirectional drive
    omni_wheels = [];
    for n=1:1:size(smooth_path,1)-1
        wheels = OmniDriveIK(r, L, smooth_path(n,:), smooth_path(n+1,:), Dt); 
        omni_wheels = [omni_wheels; [wheels.w1 wheels.w2 wheels.w3]];     
    end
    omni_wheels(end+1,:) = zeros(1, size(omni_wheels, 2)); % TODO should end in vles=0?
    writematrix(omni_wheels, OMNI_FILE);

    if (DEBUG)
        figure
        plot(smooth_path(:,1), omni_wheels(:,1)) 
        hold on
        plot(smooth_path(:,1), omni_wheels(:,2))
        hold on
        plot(smooth_path(:,1), omni_wheels(:,3))
        grid on
        legend('w1', 'w2', 'w3')
        title('Omnidirectional')
    end

end

%% ADDITIONAL FUNCTIONS

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

function inv_k = DiffDriveIK(R, L, start_pos, end_pos, dt)
% DiffDriveIK Inverse Kinematics for the Differential Drive robot
%   inv_k = DiffDriveIK(R, L, start_pos, end_pos, dt)
%   RESTRICTION: wl ~= wr
%
%   R: wheel radius
%   L: wheel base
%   start_pos: starting pose [x0, y0, theta0]
%   end_pos: ending pose [x1, y1, theta1]
%   dt: delta time

    X0 = start_pos(1);
    % Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    % Y = end_pos(2);
    TH = end_pos(3);

    if ((TH - TH0) ~= 0)
        inv_k.wl = (-L.*TH.*sin(TH - TH0)/2 + L.*TH0.*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));
        inv_k.wr = (L.*(TH - TH0).*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));
    else
        inv_k.wl = (-L.*TH + L.*TH0 + 2*X - 2*X0)./(2*R.*dt);
        inv_k.wr = (L.*TH - L.*TH0 + 2*X - 2*X0)./(2*R.*dt);
    end

end

function inv_k = OmniDriveIK(R, L, start_pos, end_pos, dt)
% OmniDriveIK Inverse Kinematics for the Omnidirectional robot
%   inv_k = OmniDriveIK(R, L, start_pos, end_pos, dt)
%   RESTRICTION: w1 ~= -w2 + w3
%
%   R: wheel radius
%   L: wheel base
%   start_pos: starting pose [x0, y0, theta0]
%   end_pos: ending pose [x1, y1, theta1]
%   dt: delta time

    X0 = start_pos(1);
    Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    Y = end_pos(2);
    TH = end_pos(3);

    inv_k.w1 = L.*TH./(R.*dt) - L.*TH0./(R.*dt) - Y./(R.*dt) + Y0./(R.*dt);
    inv_k.w2 = L.*TH./(R.*dt) - L.*TH0./(R.*dt) + sqrt(3)*X./(2*R.*dt) - sqrt(3)*X0./(2*R.*dt) + Y./(2*R.*dt) - Y0./(2*R.*dt);
    inv_k.w3 = -L.*TH./(R.*dt) + L.*TH0./(R.*dt) + sqrt(3)*X./(2*R.*dt) - sqrt(3)*X0./(2*R.*dt) - Y./(2*R.*dt) + Y0./(2*R.*dt);
    
end

function inv_k = TricycleIK(R, L, start_pos, end_pos, dt)
% TricycleIK Inverse Kinematics for the Tricyle robot
%   inv_k = TricycleIK(R, L, start_pos, end_pos, dt)
%   RESTRICTION: wt ~= 0 & alpha ~= 0
%
%   R: wheel radius
%   L: wheel base
%   start_pos: starting pose [x0, y0, theta0]
%   end_pos: ending pose [x1, y1, theta1]
%   dt: delta time

    X0 = start_pos(1);
    % Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    % Y = end_pos(2);
    TH = end_pos(3);

    if ((X - X0 ~= 0) && (TH - TH0 ~= 0))
        inv_k.alpha = atan(L.*sin(TH - TH0)./(X - X0));
        inv_k.wt = (TH - TH0).*(X - X0)./(R.*dt.*sin(TH - TH0));
    else
        inv_k.alpha = atan(L.*(TH - TH0)./(X - X0));
        inv_k.wt = (X - X0)./(R.*dt);
    end

end


