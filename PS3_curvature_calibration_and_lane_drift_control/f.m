function dxdt = f(x,u,t)
    
    dxdt = zeros(3,1);
    
    dxdt(1) = x(4)*cos(x(3)); % dx/dt
    dxdt(2) = x(4)*sin(x(3)); % dy/dt
    dxdt(3) = u(1)*x(4); % dpsi/dt
    dxdt(4) = u(2); % scalar acceleration dspeed/dt (=0 with our controller)
   
end