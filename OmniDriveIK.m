function [w1, w2, w3] = OmniDriveIK(r, L, vx, vy, w)
    %
    % Omnidirectional Drive Inverse Kinematics on the local
    % robot frame
    %

    M = [0 r/sqrt(3) -r/sqrt(3)
        -2*r/3 r/3 r/3
        r/(3*L) r/(3*L) r/(3*L)];

    w = M \ [vx vy w]';
    w1 = w(1);
    w2 = w(2);
    w3 = w(3);
    % TODO check sentidos
end