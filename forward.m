function T = forward(param)


theta1 = param(1);
theta2 = param(2);
d_3 = param(3);
theta4 = param(4);
theta5 = param(5);
theta6 = param(6);


A_1 = [cos(theta1) 0 -sin(theta1) 0; 
       sin(theta1) 0 cos(theta1) 0;
       0 -1 0 0;
       0 0 0 1];
A_2 = [cos(theta2) 0 sin(theta2) 0; 
       sin(theta2) 0 -cos(theta2) 0; 
       0 1 0 6.375*0.0254; 
       0 0 0 1];
A_3 = [1 0 0 0; 
       0 1 0 0; 
       0 0 1 d_3; 
       0 0 0 1];
A_4 = [cos(theta4) 0 -sin(theta4) 0; 
       sin(theta4) 0 cos(theta4) 0; 
       0 -1 0 0; 
       0 0 0 1];
A_5 = [cos(theta5) 0 sin(theta5) 0;
       sin(theta5) 0 -cos(theta5) 0;
       0 1 0 0;
       0 0 0 1];
A_6 = [cos(theta6) -sin(theta6) 0 0;
       sin(theta6) cos(theta6) 0 0;
       0 0 1 0;
       0 0 0 1];

T = A_1*A_2*A_3*A_4*A_5*A_6;

euler_phi = atan2(T(2, 3), T(1, 3));
euler_theta = acos(T(3, 3));
euler_psi = atan2(-T(2, 3), T(1, 3));

