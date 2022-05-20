function [wr, wl] = DiffDriveIK(r, L, vx, vy, w)
    %
    % Differential Drive Inverse Kinematics on the local
    % robot frame
    %

    M = [r/2 r/2
        0 0
        -r/L r/L];

    w = M \ [vx vy w]';
    wr = w(2);
    wl = w(1);
    % TODO check sentidos
end