function [] = TricycleIK(r, L, start_pos, end_pos, dt)

    delta = end_pos - start_pos;

    X = delta(1);
    theta = delta(3);
    TH=theta;
    k=0;
    r_t=r;
    t = dt;

    w_t= (X*theta)/(r_t*t*sin(theta))
    alpha= atan((L*TH*sin(theta))/(X*theta)) + pi*k

    w_t= (X*theta)/(r_t*t*sin(theta))
    alpha= atan((L*TH*sin(theta))/(X*theta))

    w_t= (TH*X)/(r_t*t*sin(TH))
    alpha= pi + atan((L*sin(TH))/X)
end