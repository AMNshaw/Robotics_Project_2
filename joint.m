function joint(thetaA, thetaB, thetaC)

structure;
A = s.A;
B = s.B;
C = s.C;

tacc = 0.2;
T = 0.5;
r = (T-tacc)/T;

% Find the point at which start to accelerate 
thetaAp = thetaA + (thetaB-thetaA)*r;

% Compute deltaB and deltaC
delta_C = thetaC-thetaB;
delta_B = thetaAp-thetaB;

% Motion start A->B->C, and find zaxis
theta_p = [];
theta_v = [];
theta_a = [];

for t = -T:0.002:T
    % Linear portion
    if t < -tacc
        theta_p = [theta_p, (thetaB-thetaA)*(t+0.5)/T + thetaA];
        theta_v = [theta_v, (thetaB-thetaA)/T];
        theta_a = [theta_a, zeros(6,1)];
    elseif t > tacc
        theta_p = [theta_p, delta_C*t/T + thetaB];
        theta_v = [theta_v, delta_C/T];
        theta_a = [theta_a, zeros(6,1)];
        
    % Transition portion
    else
        h = (t+tacc)/(2*tacc);
        theta_p = [theta_p, ((delta_C*tacc/T + delta_B)*(2-h)*h^2 - 2*delta_B)*h + delta_B + thetaB];
        theta_v = [theta_v, ((delta_C*tacc/T + delta_B)*(1.5-h)*2*h^2 - delta_B)/tacc];
        theta_a = [theta_a, ((delta_C*tacc/T + delta_B)*(1-h)*3*h)/tacc^2];        
    end
end

% plot the result
t = -0.5:0.002:0.5;
t1 = -0.5:0.002:-0.2-0.002;
t2 = -0.2:0.002:0.2;
t3 = 0.2+0.002:0.002:0.5;

figure
for i = 1:6
    subplot(3, 2, i), plot(t,theta_p(i,:)*180/pi);
    title(['Joint ', num2str(i)]), xlabel('Time (sec)'); 
    if i ~= 3
        ylabel('Angle (degree)');
    else
        ylabel('Position (m)');
    end
end

figure
for i = 1:6
    subplot(3, 2, i), plot(t,theta_v(i,:)*180/pi);
    title(['Joint ', num2str(i)]), xlabel('Time(sec)'); 
    if i ~= 3
        ylabel('Angular velocity (degree/sec^2)');
    else
        ylabel('Velocity (m/sec)');
    end
end

figure
for i = 1:6
    subplot(3, 2, i), plot(t,theta_a(i,:)*180/pi);
    title(['Joint ', num2str(i)]), xlabel('Time (sec)'); 
    if i ~= 3
        ylabel('Angular Acceleration (degree)');
    else
        ylabel('Acceleration (m/sec^2)');
    end
end

p = [];
zAxis = [];
for i = theta_p
    T = forward(i);
    p = [p T(1:3,4)];
    temp = T*[0 0 0.1 1]';
    zAxis = [zAxis temp(1:3,1)];
end

figure
plot3(p(1,1:length(t1)), p(2,1:length(t1)), p(3,1:length(t1)), 'color', [138 43 226]/255)
hold on
plot3(p(1,length(t1)+1:length(t1)+length(t2)+1), p(2,length(t1)+1:length(t1)+length(t2)+1), p(3,length(t1)+1:length(t1)+length(t2)+1), 'm')
plot3(p(1,end:-1:end-length(t3)+1), p(2,end:-1:end-length(t3)+1), p(3,end:-1:end-length(t3)+1), 'c')

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
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.1,0.5,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.05, B(2,4)-0.1, B(3,4)-0.05, 'B(0.3,0.3,0.2)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.3,-0.25,0.25)')

hold off, axis equal
xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Joint Motion')
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');

figure
plot3(p(1,:),p(2,:),p(3,:), 'LineWidth', 2, 'color', [138 43 226]/255), hold on

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
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.1,0.2,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.05, B(3,4)-0.01, 'B(0.3,0.2,0.1)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.1,0.2,-0.3)')

for i = 1:length(zAxis)
    plot3([p(1,i) zAxis(1,i)], [p(2,i) zAxis(2,i)], [p(3,i) zAxis(3,i)], 'color', [135 206 235]/255);
end

xlabel('X-axis(m)'), ylabel('Y-axis(m)'), zlabel('Z-axis(m)')
title('3D path of Joint Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');

