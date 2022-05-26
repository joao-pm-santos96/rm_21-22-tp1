function inv_k = DiffDriveIK(R, L, start_pos, end_pos, dt)

    X0 = start_pos(1);
    % Y0 = start_pos(2);
    TH0 = start_pos(3);

    X = end_pos(1);
    % Y = end_pos(2);
    TH = end_pos(3);
  
    inv_k.wl = (-L.*TH.*sin(TH - TH0)/2 + L.*TH0.*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));
    inv_k.wr = (L.*(TH - TH0).*sin(TH - TH0)/2 + TH.*X - TH.*X0 - TH0.*X + TH0.*X0)./(R.*dt.*sin(TH - TH0));

end