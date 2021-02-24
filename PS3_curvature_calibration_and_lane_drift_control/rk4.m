function [slope,dt_next] = rk4(f,u,x,t,dt)
    s1 = f(x,u(x,t),t);
    s2 = f(x+dt*s1/2,u(x+dt*s1/2,t+dt/2),t+dt/2);
    s3 = f(x+dt*s2/2,u(x+dt*s2/2,t+dt/2),t+dt/2);
    s4 = f(x+dt*s3,u(x+dt*s3,t+dt),t+dt);
    slope = (s1+2*s2+2*s3+s4)/6;
    alpha = 0.95;
    error = norm((x+dt*slope) - (x+dt*slope/2 + s2*dt/2));
    R=(1e-3+(1e-4)*norm(x+slope*dt)/abs(error))^(1/(4+1));
    dt_next = alpha*R*dt;
end