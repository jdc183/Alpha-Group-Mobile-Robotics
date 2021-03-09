function input = controller(x,t)
    global K_offset K_psi
    d_offset = x(2);
    psi = x(3);
    rho = - K_offset * d_offset - K_psi * psi;
    rho1 = - K_offset * d_offset;
    rho2 = - K_psi * psi;
    
    rho = rho1 + rho2;
    
    acc = 0;
    
    input = [rho acc];
end