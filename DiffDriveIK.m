function inv_k = DiffDriveIK(R, L, start_pos, end_pos, dt)
% DiffDriveIK Inverse Kinematics for the Differential Drive robot
%   inv_k = DiffDriveIK(R, L, start_pos, end_pos, dt)
%   RESTRICTION: wl ~= wr
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

    if ((TH - TH0) ~= 0)
        inv_k.wl = (-L.*TH.*sin(TH - TH0)/2 + L.*TH0.*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));
        inv_k.wr = (L.*(TH - TH0).*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));
    else
        inv_k.wl = (-L.*TH + L.*TH0 + 2*X - 2*X0)./(2*R.*dt);
        inv_k.wr = (L.*TH - L.*TH0 + 2*X - 2*X0)./(2*R.*dt);
    end

end