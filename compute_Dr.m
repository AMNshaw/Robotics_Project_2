function Dr = compute_Dr(x,y,z,psi,theta,phi,r)

V_rtheta = 1 - cos(r*theta);

Tr = [1, 0, 0, r*x;
      0, 1, 0, r*y;
      0, 0, 1, r*z;
      0, 0, 0, 1];
  
R_ar = [sin(psi)^2*V_rtheta + cos(r*theta), -sin(psi)*cos(psi)*V_rtheta, cos(psi)*sin(r*theta), 0;
        -sin(psi)*cos(psi)*V_rtheta, cos(psi)^2*V_rtheta + cos(r*theta), sin(psi)*sin(r*theta), 0;
        -cos(psi)*sin(r*theta), -sin(psi)*sin(r*theta), cos(r*theta), 0;
        0, 0, 0, 1;];
    
R_or = [cos(r*phi), -sin(r*phi), 0, 0;
        sin(r*phi), cos(r*phi), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;];
    
Dr = Tr*R_ar*R_or;