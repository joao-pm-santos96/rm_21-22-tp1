function wheels = OmniDriveIK(r, L, start_pos, end_pos, dt)

    delta = end_pos - start_pos;
    X=delta(1);
    Y=delta(2);
    theta=start_pos(3);
    TH=theta;
    R=r;
    t=dt;

%     wheels = [-(delta(2) - L*delta(3))/(r*dt) %w1
%         (delta(2) + 3^(1/2)*delta(1) + 2*L*delta(3))/(2*r*dt) % w2
%         -(delta(2) - 3^(1/2)*delta(1) + 2*L*delta(3))/(2*r*dt) ]'; % w3

    wheels = [(X*sin(theta) - Y*cos(theta) + L*TH)/(R*t)
        (Y*cos(theta) - X*sin(theta) + 2*L*TH + 3^(1/2)*X*cos(theta) + 3^(1/2)*Y*sin(theta))/(2*R*t)
        (X*sin(theta) - Y*cos(theta) - 2*L*TH + 3^(1/2)*X*cos(theta) + 3^(1/2)*Y*sin(theta))/(2*R*t)];
    
    
    
end