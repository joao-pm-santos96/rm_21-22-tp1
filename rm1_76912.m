function rm1_76912(N, Dt, r, L, Vn, Wn, vel)

    % TODO add all funtions to this file!!

%     arguments
% 
%     end

    %% Default Inputs
    if nargin < 7
        % velocidade desejada
        vel = 5; %m/s
    end
    if nargin < 6
        % incerteza (sigma) na velocidade angular a impor ao robot
        Wn = 0.1; %rad/s
    end
    if nargin < 5
        % incerteza (sigma) na velocidade linear a impor ao robot
        Vn = 0.1; %m/s
    end
    if nargin < 4
        % separacao/afastamento das rodas
        L = 1; %m
    end
    if nargin < 3
        % raio das rodas do robot
        r = 0.15; %m
    end
    if nargin < 2
        % intervalo de tempo de amostragem dos sensores
        Dt = 1; %s
    end
    if nargin < 1
        % numero de farois
        N = 4;
    end

    %% Constants 
    INITIAL_POSE = [0,0,0];
    LOC_FILE = 'loc_76912.txt';
    DD_FILE = 'DD_76912.txt';
    TRI_FILE = 'TRI_76912.txt';
    OMNI_FILE = 'OMNI_76912.txt';
    DEBUG = true;

    if (DEBUG)
        close all
    end
    
    %% Step 1: Compute all path points
    if (DEBUG)
        disp('Step 1')
    end

    beacons = BeaconDetection(N);

    % Get all known points
    beacon_poses = [];
    for n=1:1:N
        beacon_poses = [beacon_poses; beacons(n).X, beacons(n).Y, beacons(n).a];
    end
    known_poses = [INITIAL_POSE; beacon_poses];

    % Get intermediate points per section
    all_poses = INITIAL_POSE;
    for n=1:1:N
        % Compute distance between two points
        p0 = known_poses(n,:);
        p1 = known_poses(n+1,:);
        d = norm(p1-p0);
        
        % Get n points
        n_points = round(d / (vel * Dt));

        deltas = (1:1:n_points)/n_points;
        step_poses = p0+deltas'*(p1-p0);
        all_poses = [all_poses; step_poses];
    end

    % Get smooth path coordinates
    yq = pchip(known_poses(:,1), known_poses(:,2), all_poses(:,1));
    smooth_path = [all_poses(:,1), yq];

    % Get smooth path orientations
    orients = [];
    for n=1:1:size(smooth_path,1)-1
        p0 = smooth_path(n,:);
        p1 = smooth_path(n+1,:);
        v = p1 - p0;
        orients = [orients; atan2(v(2),v(1))];
    end
    orients(end+1)=orients(end); % Keep one-to-last until end

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

    control_inputs = []; % TODO check if they make sense...
    for n=1:1:size(smooth_path,1)-1
        p0 = smooth_path(n,:);
        p1 = smooth_path(n+1,:);
        v = p1 - p0;
        lin_vel = norm(v(1:2)) / Dt;
        ang_vel = v(3) / Dt;
        control_inputs = [control_inputs; [lin_vel, ang_vel]];
    end
    % TODO check if should stop at end and how/what to do it
    control_inputs(end+1,:) = [0, 0];

    %%% DEBUG %%%
    if (DEBUG)
        figure
        plot(smooth_path(:,1), control_inputs(:,1), 'bo')
        grid on
        title('Linear Velocity')
    
        figure
        plot(smooth_path(:,1), control_inputs(:,2), 'bo')
        grid on
        title('Angular Velocity')
    end
    %%%%%%%%%%%%%

    %% Step 3: Extended Kalamn Filter 
    if (DEBUG)
        disp('Step 3')
    end

    % Initial values
    % TODO check this...
    P_t = 0.01 * eye(3);

    Q = [Vn^2 0
        0 Wn^2];

    % Get all beacon positions
    ekf_loc = INITIAL_POSE;
    ekf_p  = P_t;

    for n=2:1:size(control_inputs,1)

        state_t = smooth_path(n-1,:);
        control_t = control_inputs(n-1,:);
        beacons = BeaconDetection(N, smooth_path(n,:));
        obs_t1 = [beacons(:).d; beacons(:).a]';
        landmarks = beacon_poses(:,1:2);

        % Deal with NaNs
        [rows, ~] = find(isnan(obs_t1));
        nan_rows = unique(rows);
        b_noises = [beacons(:).dn; beacons(:).an];

        obs_t1(nan_rows,:) = [];
        landmarks(nan_rows,:) = [];
        b_noises(:,nan_rows) = [];

        b_noises = reshape(b_noises,1,[]);
        
        R = diag(b_noises);

        [state_t1, P_t1] = EKF(state_t, P_t, control_t, obs_t1, landmarks, Dt, Q, R);

        state_t = state_t1;
        P_t = P_t1;

        ekf_loc = [ekf_loc; state_t];
        ekf_p = [ekf_p; P_t];
    end

    if (DEBUG)
        figure
        plot(smooth_path(:,1),smooth_path(:,2),'g-')
        hold on
        plot(ekf_loc(:,1), ekf_loc(:,2),'r-.')
        title('EKF')
        grid on
        %TODO draw uncertainty ellipsis
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
    for n=2:1:size(smooth_path,1)

        wheels = DiffDriveIK(r, L, smooth_path(n-1,:), smooth_path(n,:), Dt); % TODO should end in vles=0?
        diff_wheels = [diff_wheels; wheels]; % TODO check order of wheels and NaNs

    end
    writematrix(diff_wheels, DD_FILE);
    


    % Omnidirectional drive
    omni_wheels = [];
    for n=2:1:size(smooth_path,1)

        wheels = OmniDriveIK(r, L, smooth_path(n-1,:), smooth_path(n,:), Dt); % TODO should end in vles=0?
        omni_wheels = [omni_wheels; wheels]; % TODO check order of wheels and NaNs
    
    end
    writematrix(omni_wheels, OMNI_FILE);







    



end

% % Differential drive
% diff_wheels = [];
% for n=1:1:size(control_inputs,1)
% 
%     alpha = smooth_path(n,3);
%     transform = [cos(alpha) 0
%             sin(alpha) 0
%             0 1];
% 
%     vels = transform * control_inputs(n,:)';
%     [wr, wl] = DiffDriveIK(r,L,vels(1),vels(2),vels(3));
%     diff_wheels = [diff_wheels; [wr, wl]];
% end
% writematrix(diff_wheels, DD_FILE);
% 
% if (DEBUG)
%     figure
%     plot(smooth_path(:,1), diff_wheels(:,1))
%     hold on
%     plot(smooth_path(:,1), diff_wheels(:,2))
%     legend('Right wheel', 'Left wheel')
%     title('Differential Drive')
%     grid on
% end
% 
% % Tricycle
% tri_wheels = [];
% for n=1:1:size(control_inputs,1)
%     disp('Do me!!')
% end
% writematrix(tri_wheels, TRI_FILE)
% 
% %     if (DEBUG)
% %         figure
% %         plot(smooth_path(:,1), tri_wheels(:,1))
% %         hold on
% %         plot(smooth_path(:,1), tri_wheels(:,2))
% %         legend('Wheel speed', 'Steering angle')
% %         title('Tricycle')
% %         grid on
% %     end
% 
% % Omnidirectional drive
% omni_wheels = [];
% for n=1:1:size(control_inputs,1)
% 
%     alpha = smooth_path(n,3);
% %         transform = [cos(alpha) -sin(alpha) 0
% %             sin(alpha) cos(alpha) 0
% %             0 0 1];
%     transform = [cos(alpha) 0
%             sin(alpha) 0
%             0 1]; % TODO is it this or the above?
%     
%     vels = transform * control_inputs(n,:)';
%     [w1, w2, w3] = OmniDriveIK(r,L,vels(1),vels(2),vels(3));
%     omni_wheels = [omni_wheels; [w1, w2, w3]];
% end
% writematrix(omni_wheels, OMNI_FILE);
% 
% if (DEBUG)
%     figure
%     plot(smooth_path(:,1), omni_wheels(:,1))
%     hold on
%     plot(smooth_path(:,1), omni_wheels(:,2))
%     hold on
%     plot(smooth_path(:,1), omni_wheels(:,3))
%     legend('Wheel 1', 'Wheel 2', 'Wheel 3')
%     title('Omnidrive')
%     grid on
% end
