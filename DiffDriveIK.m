function wheels = DiffDriveIK(r, L, start_pos, end_pos, dt)
  
    delta=end_pos-start_pos;
    X=delta(1);
    theta=delta(3);
   
    wheels = [(2*X*theta + L*theta*sin(theta))/(2*r*dt*sin(theta)) % right
        (2*X*theta - L*theta*sin(theta))/(2*r*dt*sin(theta))]'; % left
     
end