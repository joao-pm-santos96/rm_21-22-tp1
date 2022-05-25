function [x, y, theta] = TricyckeFK(r, L, w, alpha, start_pos, dt)

    x_dot = w * r;
    y_dot = 0;
    theta_dot = w * r * tan(alpha) / L;

    pos = [x_dot y_dot theta_dot] * dt;

    end_pos = pos + start_pos
    x = end_pos(1);
    y = end_pos(2);
    theta = end_pos(3);

end