function [vl, vr] = DiffDriveIK(r, L, dx, dtheta, dt)
    w = dtheta / dt
    v = dx / sin(dt*w) * w 

    coeffs = [1/L -1/L
        0.5 0.5];

    inv(coeffs) * [w v]' / r
end