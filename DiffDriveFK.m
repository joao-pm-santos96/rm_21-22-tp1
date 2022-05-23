function [x,y,theta] = DiffDriveFK(r, L, wr, wl, x0, y0, theta0, dt)

    M = [r/2 r/2
        0 0
        -r/L r/L];

    trans = [cos(theta0) -sin(theta0) 0
        sin(theta0) cos(theta0) 0
        0 0 1];

    pos = trans * M * [wl wr]' * dt;

    x = pos(1) + x0;
    y = pos(2) + y0;
    theta = pos(3) + theta0;

end