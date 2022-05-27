function rm1_76912(N, Dt, r, L, Vn, Wn, vel)

    % TODO add all funtions to this file!!

    %% Default Inputs
    arguments
        N = 4
        Dt = 1
        r = 0.15
        L = 1
        Vn = 0.1
        Wn = 0.1
        vel = 5
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
    orients = [INITIAL_POSE(3)]; 
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
    P_t = 0.01 * eye(3); % Initial estimate
    Q = [Vn^2 0
        0 Wn^2]; % Covariance for the zero-mean Gaussin noise

    % Get all beacon positions
    ekf_loc = INITIAL_POSE;
    ekf_p  = P_t;
    
    % Build sigma vectors
    % TODO should be zeros?

    % s_motion = [Vn, Wn]; % uncertainty (sigma) for the motion model
    % s_sensor = [beacons(1).dn; beacons(1).an]; % uncertainty (sigma) for the sensor model

    s_motion = zeros(1,2);
    s_sensor = zeros(1,2);

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

        [state_t1, P_t1] = EKF(state_t, P_t, control_t, obs_t1, landmarks, Dt, Q, R, s_motion, s_sensor);

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
        diff_wheels = [diff_wheels; [wheels.wr wheels.wl]]; 

    end
    writematrix(diff_wheels, DD_FILE);
    
    % Tricyle
    tri_wheels = [];
    for n=2:1:size(smooth_path,1)

        wheels = TricycleIK(r, L, smooth_path(n-1,:), smooth_path(n,:), Dt); % TODO should end in vles=0?
        tri_wheels = [tri_wheels; [wheels.wt wheels.alpha]]; 
    
    end
    writematrix(tri_wheels, TRI_FILE);

    % Omnidirectional drive
    omni_wheels = [];
    for n=2:1:size(smooth_path,1)

        wheels = OmniDriveIK(r, L, smooth_path(n-1,:), smooth_path(n,:), Dt); % TODO should end in vles=0?
        omni_wheels = [omni_wheels; [wheels.w1 wheels.w2 wheels.w3]]; 
    
    end
    writematrix(omni_wheels, OMNI_FILE);

    disp(smooth_path(end-1,:))
    disp(smooth_path(end,:))

end