function inv_k = TricycleIK(R, L, start_pos, end_pos, dt)
% TricycleIK Inverse Kinematics for the Tricyle robot
%   inv_k = TricycleIK(R, L, start_pos, end_pos, dt)
%
%   R: wheel radius
%   L: wheel base
%   start_pos: starting pose [x0, y0, theta0]
%   end_pos: ending pose [x1, y1, theta1]
%   dt: delta time

    X0 = start_pos(1);
    % Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    % Y = end_pos(2);
    TH = end_pos(3);

    inv_k.alpha = atan(L.*sin(TH - TH0)./(X - X0));
    inv_k.wt = (TH - TH0).*(X - X0)./(R.*dt.*sin(TH - TH0));

end