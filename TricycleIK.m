function TricycleIK(R, L, start_pos, end_pos, dt)

    X0 = start_pos(1);
    Y0 = start_pos(2)
    TH0 = start_pos(3);

    X = end_pos(1);
    Y = end_pos(2)
    TH = end_pos(3);

    inv_k = {alpha: atan(L.*sin(TH - TH0)./(X - X0)), 
    w: (TH - TH0).*(X - X0)./(R.*dt.*sin(TH - TH0))};
    
end