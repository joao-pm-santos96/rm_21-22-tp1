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
    plotBeacons(BeaconDetection(N)) 

    % Get path coordinates
    curr_pose = INITIAL_POSE;
    B = BeaconDetection(N, curr_pose);
    n_points = [];

    for n=1:1:N
        % Get beacon n position
        beacon_pose = [B(n).X, B(n).Y, B(n).a];

        % Compute distance to current pose
        d = norm(beacon_pose(1:2) - curr_pose(1:2));
        
        % Store n interval points
        n_points = [n_points, round(d / (velocity * Dt))];

        % Update current pose
        curr_pose = beacon_pose;
    end











end

%% Aux Functions
function plotBeacons(B)
    xx = [];
    yy = [];

    for n=1:1:size(B,2)
        xx = [xx, B(n).X];
        yy = [yy, B(n).Y];
    end

    plot(xx,yy,'bo');
    hold on;

    xx = [0, xx];
    yy = [0, yy];

    plot(xx,yy,'r--');
    grid on;
end