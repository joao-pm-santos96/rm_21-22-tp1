function TricycleIK(r, L, x, y, theta, dt)

    alpha = arcsin(theta / (ws * r) * L)

    x = ws * r * cos(alpha) * cos(theta) * dt

end