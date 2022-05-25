function wheels = OmniDriveIK(r, L, start_pos, end_pos, dt)

    delta = end_pos - start_pos;

    wheels = [-(delta(2) - L*delta(3))/(r*dt) %w1
        (delta(2) + 3^(1/2)*delta(1) + 2*L*delta(3))/(2*r*dt) % w2
        -(delta(2) - 3^(1/2)*delta(1) + 2*L*delta(3))/(2*r*dt) ]'; % w3
end