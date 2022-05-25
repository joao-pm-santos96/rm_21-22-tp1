function vel = OmniDriveFK(r, L, wheels, start_pos)

    w1 = wheels(1);
    w2 = wheels(2);
    w3 = wheels(3);

    theta = start_pos(3);

    M = [       0, (3^(1/2)*r)/3, (3^(1/2)*r)/3
        -(2*r)/3,           r/3,          -r/3
         r/(3*L),       r/(3*L),      -r/(3*L)];

    trans = [cos(theta) -sin(theta) 0
        sin(theta) cos(theta) 0
        0 0 1];

    vel = trans * M * [w1 w2 w3]';

end