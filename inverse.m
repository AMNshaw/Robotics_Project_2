function thetas = inverse(T)

n = T(1:3, 1);
o = T(1:3, 2);
a = T(1:3, 3);
p = T(1:3, 4);

theta2 = zeros(8, 1);
theta1 = zeros(8, 1);
theta4 = zeros(8, 1);
theta5 = zeros(8, 1);
theta6 = zeros(8, 1);
d_3 = zeros(2, 1);

% convert inch into m

theta2(1) = atan2(sqrt(p(1)^2 + p(2)^2 - (6.375*0.0254)^2), p(3));
theta2(2) = atan2(-sqrt(p(1)^2 + p(2)^2 - (6.375*0.0254)^2), p(3));
theta2(3) = atan2(sqrt(p(1)^2 + p(2)^2 - (6.375*0.0254)^2), p(3)) - pi;
theta2(4) = atan2(-sqrt(p(1)^2 + p(2)^2 - (6.375*0.0254)^2), p(3)) + pi;

for i = 1:4
    theta2(i+4) = theta2(i);
end

for i = 1:4
    d_3(i) = p(3)/cos(theta2(i));
    theta1(i) = atan2(d_3(i)*sin(theta2(i)), 6.375*0.0254) - atan2(p(1), p(2));
    d_3(i+4) = d_3(i);
    theta1(i+4) = theta1(i);
end

for i = 1:4
    theta4(i) = atan2((-sin(theta1(i))*a(1) + cos(theta1(i))*a(2)), (cos(theta1(i))*cos(theta2(i))*a(1) + cos(theta2(i))*sin(theta1(i))*a(2) - sin(theta2(i))*a(3)));
    if(theta4(i) > 0)
        theta4(i + 4) = theta4(i) - pi;
    else
        theta4(i + 4) = theta4(i) + pi;
    end
end

for i = 1:8
      theta5(i) = atan2( (cos(theta1(i))*cos(theta2(i))*cos(theta4(i)) - sin(theta1(i))*sin(theta4(i)))*a(1) + (cos(theta1(i))*sin(theta4(i)) + cos(theta2(i))*cos(theta4(i))*sin(theta1(i)))*a(2) -cos(theta4(i))*sin(theta2(i))*a(3) ...
          , cos(theta1(i))*sin(theta2(i))*a(1) + sin(theta1(i))*sin(theta2(i))*a(2)+cos(theta2(i))*a(3));
end

input_stuck = zeros(8, 1);

for i = 1:4
 theta6(i) = atan2(cos(theta1(i))*sin(theta2(i))*o(1) + sin(theta1(i))*sin(theta2(i))*o(2) + cos(theta2(i))*o(3) ...
                   ,-(cos(theta1(i))*sin(theta2(i))*n(1) + sin(theta1(i))*sin(theta2(i))*n(2) + cos(theta2(i))*n(3)));
    if theta6(i) > 0
        theta6(i+4) = theta6(i) - pi;
    else
        theta6(i+4) = theta6(i) + pi;
    end
end

for i = 1:8
    %fprintf("\nSolution %i: \n", i);
    %fprintf("%f, %f, %f, %f, %f, %f \n", theta1(i)* 180/pi, theta2(i)* 180/pi, d_3(i), theta4(i)* 180/pi, theta5(i)* 180/pi, theta6(i)* 180/pi);
    if abs(theta1(i)) > 160 * pi/180
    %    fprintf("θ_1 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    if abs(theta2(i)) > 125 * pi/180
    %    fprintf("θ_2 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    if abs(d_3(i)) > 30*0.0254
    %    fprintf("d_3 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    if abs(theta4(i)) > 140 * pi/180
    %    fprintf("θ_4 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    if abs(theta5(i)) > 100 * pi/180
    %    fprintf("θ_5 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    if abs(theta6(i)) > 260 * pi/180
    %    fprintf("θ_6 out of range !\n");
        input_stuck(i) = input_stuck(i) - 1;
    end
    
end
fprintf("\n");

[least_stuck ]= max(input_stuck);
theta_1 = [];
theta_2 = [];
d3 = [];
theta_4 = [];
theta_5 = [];
theta_6 = [];
for i = 1:8
    if input_stuck(i) == least_stuck
        theta_1 = [theta_1; theta1(i)];
        theta_2 = [theta_2; theta2(i)];
        d3 = [d3; d_3(i)];
        theta_4 = [theta_4; theta4(i)];
        theta_5 = [theta_5; theta5(i)];
        theta_6 = [theta_6; theta6(i)];
    end
end

thetas = [theta_1, theta_2, d3, theta_4, theta_5, theta_6];




