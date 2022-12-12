clear all; close all;clc;

structure;
A = s.A;
B = s.B;
C = s.C;


% Find inverse kinematics of all joints that is availible

thetaA = inverse(A);
thetaB = inverse(B);
thetaC = inverse(C);

thetaA = thetaA';
thetaB = thetaB';
thetaC = thetaC';

for i = 1:size(thetaA, 2)
    for j = i:size(thetaB, 2)
        for k = 1:size(thetaC, 2)
            fprintf("Combination %d: \n", i);
            fprintf("A: (θ1, θ2, d3, θ4, θ5, θ6):\n(%f, %f, %f, %f, %f, %f)\n\n", thetaA(1, i)*180/pi, thetaA(2, i)*180/pi, thetaA(3, i), thetaA(4, i)*180/pi, thetaA(5, i)*180/pi, thetaA(6, i)*180/pi);
            fprintf("B: (θ1, θ2, d3, θ4, θ5, θ6):\n(%f, %f, %f, %f, %f, %f)\n\n", thetaB(1, j)*180/pi, thetaB(2, j)*180/pi, thetaB(3, j), thetaB(4, j)*180/pi, thetaB(5, j)*180/pi, thetaB(6, j)*180/pi);
            fprintf("C: (θ1, θ2, d3, θ4, θ5, θ6):\n(%f, %f, %f, %f, %f, %f)\n\n", thetaC(1, k)*180/pi, thetaC(2, k)*180/pi, thetaC(3, k), thetaC(4, k)*180/pi, thetaC(5, k)*180/pi, thetaC(6, k)*180/pi);
            joint(thetaA(:, i), thetaB(:, j), thetaC(:, k));
            prompt = "System pause, press Enter to continue";
            input(prompt);
            close all;
        end
    end
end