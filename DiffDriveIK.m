function [] = DiffDriveIK(r, L, start_pos, end_pos, dt)
   
    theta_dot = (end_pos(3) - start_pos(3)) / dt;
    v = (end_pos(1) - start_pos(1)) * theta_dot / sin(theta_dot * dt);

    coeffs = [1/L -1/L
        1/2 1/2];

    vels = (coeffs \ [theta_dot v]') / r;
    wr = vels(1)
    wl = vels(2)

    % TODO
    delta=end_pos-start_pos;
    X=delta(1);
    theta=delta(3);
    TH=theta;
    t=dt;

     w_r= (2*X*theta + L*TH*sin(theta))/(2*r*t*sin(theta))
     w_l= (2*X*theta - L*TH*sin(theta))/(2*r*t*sin(theta))

     


end