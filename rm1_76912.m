function rm1_76912(N, Dt, r, L, Vn, Wn)

    %% Default Inputs
    if nargin < 6
        Wn = 0.1; %m/s
    end
    if nargin < 5
        Vn = 0.1; %m/s
    end
    if nargin < 4
        L = 1; %m
    end
    if nargin < 3
        r = 0.15; %m
    end
    if nargin < 2
        Dt = 1; %s
    end
    if nargin < 1
        N = 4;
    end

    %% Constants 
    initial_pose = [0,0,0];
    loc_file = 'loc_76912.txt';
    dd_file = 'dd_76912.txt';
    tri_file = 'tri_76912.txt';
    omni_file = 'omni_76912.txt';
    
    %% Step 1: Compute all path points
    B = BeaconDetection(N);







end