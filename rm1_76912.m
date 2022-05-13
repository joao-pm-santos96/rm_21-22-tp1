function rm1_76912(N, Dt, r, L, Vn, Wn)

    %% Default Inputs
    if nargin < 6
        % incerteza (sigma) na velocidade angular a impor ao robot
        Wn = 0.1; %m/s
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
    DD_FILE = 'dd_76912.txt';
    TRI_FILE = 'tri_76912.txt';
    OMNI_FILE = 'omni_76912.txt';
    
    %% Step 1: Compute all path points
    velocity = 5; %TODO where to get this from?
    beacons = BeaconDetection(N);

    % Get all known points
    known_poses = INITIAL_POSE;
    for n=1:1:N
        beacon = [beacons(n).X, beacons(n).Y, beacons(n).a];
        known_poses = [known_poses; beacon];
    end

    % Get intermediate points per section
    all_poses = INITIAL_POSE;
    for n=1:1:N
        % Compute distance between two points
        p0 = known_poses(n,:);
        p1 = known_poses(n+1,:);
        d = norm(p1-p0);
        
        % Get n points
        n_points = round(d / (velocity * Dt));

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
    figure
    plot(known_poses(:,1), known_poses(:,2),'bo')
    hold on
    plot(known_poses(:,1), known_poses(:,2),'r--')
    hold on
    plot(smooth_path(:,1),smooth_path(:,2),'g-')
    grid on
    %%%%%%%%%%%%%

    %% Step 2: 



end
