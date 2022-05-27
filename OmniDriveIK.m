function inv_k = OmniDriveIK(R, L, start_pos, end_pos, dt)
% OmniDriveIK Inverse Kinematics for the Omnidirectional robot
%   inv_k = OmniDriveIK(R, L, start_pos, end_pos, dt)
%
%   R: wheel radius
%   L: wheel base
%   start_pos: starting pose [x0, y0, theta0]
%   end_pos: ending pose [x1, y1, theta1]
%   dt: delta time

    X0 = start_pos(1);
    Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    Y = end_pos(2);
    TH = end_pos(3);

    inv_k.w1 = L.*TH./(R.*dt) - L.*TH0./(R.*dt) - Y./(R.*dt) + Y0./(R.*dt);
    inv_k.w2 = L.*TH./(R.*dt) - L.*TH0./(R.*dt) + sqrt(3)*X./(2*R.*dt) - sqrt(3)*X0./(2*R.*dt) + Y./(2*R.*dt) - Y0./(2*R.*dt);
    inv_k.w3 = -L.*TH./(R.*dt) + L.*TH0./(R.*dt) + sqrt(3)*X./(2*R.*dt) - sqrt(3)*X0./(2*R.*dt) - Y./(2*R.*dt) + Y0./(2*R.*dt);
    
end