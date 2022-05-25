function vel = DiffDriveFK(r, L, wr, wl, start_pos)

    M = [r/2 r/2
        0 0
        -r/L r/L];

    trans = [cos(start_pos(3)) -sin(start_pos(3)) 0
        sin(start_pos(3)) cos(start_pos(3)) 0
        0 0 1];

    pos = trans * M * [wl wr]';

    vel = [pos(1) + start_pos(1)
        pos(2) + start_pos(2)
        pos(3) + start_pos(3)];
    
end