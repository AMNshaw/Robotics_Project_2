clear all; close all;clc;
structure;

A = s.A;
B = s.B;
C = s.C;

tacc = 0.2;
T = 0.5;
r = (T-tacc)/T;

% Accelerate point
[x, y, z, psi, theta, phi] = compute_D1(A,B);
Dr = compute_Dr(x, y, z, psi, theta, phi, r);
Acc_p = A*Dr;

% Compute delta_B & delta_C
[delta_B_x, delta_B_y, delta_B_z, delta_B_psi, delta_B_theta, delta_B_phi] = compute_D1(B, Acc_p);
[delta_C_x, delta_C_y, delta_C_z, delta_C_psi, delta_C_theta, delta_C_phi] = compute_D1(B, C);
delta_C = [delta_C_x delta_C_y delta_C_z];

% To minimize the effects of additional acceleration
if abs(rad2deg(delta_C_psi-delta_B_psi)) > 90
    delta_B_psi = delta_B_psi + pi;
    if delta_B_psi > pi
        delta_B_psi = delta_B_psi - 2*pi;
    end
    delta_B_theta = -delta_B_theta;
end


% Motion start A->B->C, and find zaxis
p = [];
v = [];
a = [];
zAxis = [];

for t = -T:0.002:T
    
    % Linear portion
    if t < -0.2
        h = (t+T)/T;
        Dr = compute_Dr(x, y, z, psi, theta, phi, h);
        Dv = [x/T;
              y/T;
              z/T;
                0;];

        p = [p, A(1:3,:)*Dr(:,4)];
        v = [v, A(1:3,:)*Dv];
        a = [a, zeros(3,1)];
        
        temp = A*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
    elseif t > 0.2
        h = t/T;
        Dr = compute_Dr(delta_C_x, delta_C_y, delta_C_z, delta_C_psi, delta_C_theta, delta_C_phi, h);
        Dv = [delta_C(1)/T;
              delta_C(2)/T;
              delta_C(3)/T;
                         0;];

        p = [p, B(1:3,:)*Dr(:,4)];
        v = [v, B(1:3,:)*Dv];
        a = [a, zeros(3,1)];
        
        temp = B*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
        
    % Transition portion
    else
        h = (t+tacc)/(2*tacc);
        
        Dp = [((delta_C_x*tacc/T + delta_B_x)*(2-h)*h^2 - 2*delta_B_x)*h + delta_B_x;
              ((delta_C_y*tacc/T + delta_B_y)*(2-h)*h^2 - 2*delta_B_y)*h + delta_B_y;
              ((delta_C_z*tacc/T + delta_B_z)*(2-h)*h^2 - 2*delta_B_z)*h + delta_B_z;
              (delta_C_psi-delta_B_psi)*h + delta_B_psi;
              ((delta_C_theta*tacc/T + delta_B_theta)*(2-h)*h^2 - 2*delta_B_theta)*h + delta_B_theta;
              ((delta_C_phi*tacc/T + delta_B_phi)*(2-h)*h^2 - 2*delta_B_phi)*h + delta_B_phi;];
          
        Dv = [((delta_C_x*tacc/T + delta_B_x)*(1.5-h)*2*h^2 - delta_B_x)/tacc;
              ((delta_C_y*tacc/T + delta_B_y)*(1.5-h)*2*h^2 - delta_B_y)/tacc;
              ((delta_C_z*tacc/T + delta_B_z)*(1.5-h)*2*h^2 - delta_B_z)/tacc;
                                                          0;];
          
        Da = [(delta_C_x*tacc/T + delta_B_x)*(1-h)*3*h/tacc^2;
              (delta_C_y*tacc/T + delta_B_y)*(1-h)*3*h/tacc^2;
              (delta_C_z*tacc/T + delta_B_z)*(1-h)*3*h/tacc^2;
                                                0;];

        Dr = compute_Dr(Dp(1), Dp(2), Dp(3), Dp(4), Dp(5), Dp(6), 1);
          
        p = [p, B(1:3,:)*Dr(:,4)];
        v = [v, B(1:3,:)*Dv];
        a = [a, B(1:3,:)*Da];
        
        temp = B*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
    end    
end



% Show the position, velocity, acceleration of x,y,z
t = 0:0.002:1;

figure;
subplot(3, 1, 1), plot(t,p(1,:)*180/pi)
title('Position of x'), xlabel('Time(sec)'), ylabel('Position(m)')
subplot(3, 1, 2), plot(t,p(2,:))
title('Position of y'), xlabel('Time(sec)'), ylabel('Position(m)')
subplot(3, 1, 3), plot(t,p(3,:))
title('Position of z'), xlabel('Time(sec)'), ylabel('Position(m)')

figure;
subplot(3, 1, 1), plot(t,v(1,:))
title('Velocity of x'), xlabel('Time(sec)'), ylabel('Velocity(m/sec)')
subplot(3, 1, 2), plot(t,v(2,:))
title('Velocity of y'), xlabel('Time(sec)'), ylabel('Velocity(m/sec)')
subplot(3, 1, 3), plot(t,v(3,:))
title('Velocity of z'), xlabel('Time(sec)'), ylabel('Velocity(m/sec)')


figure;
subplot(3, 1, 1), plot(t,a(1,:))
title('Acceleration of x'), xlabel('Time(sec)'), ylabel('Acceleration(m/sec^2)')
subplot(3, 1, 2), plot(t,a(2,:))
title('Acceleration of y'), xlabel('Time(sec)'), ylabel('Acceleration(m/sec^2)')
subplot(3, 1, 3), plot(t,a(3,:))
title('Acceleration of z'), xlabel('Time(sec)'), ylabel('Acceleration(m/sec^2)')

figure;
% Show the moving path
plot3(p(1,:), p(2,:), p(3,:))
hold on

% Show the A B C and their orientation 
Ax = A*[0.1 0 0 1]';
Ay = A*[0 0.1 0 1]';
Az = A*[0 0 0.1 1]';
plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B*[0.1 0 0 1]';
By = B*[0 0.1 0 1]';
Bz = B*[0 0 0.1 1]';
plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C*[0.1 0 0 1]';
Cy = C*[0 0.1 0 1]';
Cz = C*[0 0 0.1 1]';
plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.1,0.2,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(0.3,0.2,0.1)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.25,0.1,-0.2)')

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Cartesian Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off

% Show the moving path
figure
plot3(p(1,:), p(2,:), p(3,:), 'LineWidth', 2, 'color', [138 43 226]/255)
hold on

% Show the A B C and their orientation
Ax = A*[0.1 0 0 1]';
Ay = A*[0 0.1 0 1]';
Az = A*[0 0 0.1 1]';
plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B*[0.1 0 0 1]';
By = B*[0 0.1 0 1]';
Bz = B*[0 0 0.1 1]';
plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C*[0.1 0 0 1]';
Cy = C*[0 0.1 0 1]';
Cz = C*[0 0 0.1 1]';
plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.1,0.2,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(0.3,0.2,0.1)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.1,-0.2,-0.3)')


% Show the z-axis of end-effector
for i = 1:length(zAxis)
    plot3([p(1,i) zAxis(1,i)], [p(2,i) zAxis(2,i)], [p(3,i) zAxis(3,i)],'c');
end

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Cartesian Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off