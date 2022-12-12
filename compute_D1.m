function [x, y, z, psi, theta, phi] = compute_D1(pos1, pos2)

n_1 = pos1(:,1);
o_1 = pos1(:,2);
a_1 = pos1(:,3);

n_2 = pos2(:,1);
o_2 = pos2(:,2);
a_2 = pos2(:,3);

p_1 = pos1(:,4);
p_2 = pos2(:,4);

x = dot(n_1, (p_2 - p_1));
y = dot(o_1, (p_2 - p_1));
z = dot(a_1, (p_2 - p_1));

psi = atan2(dot(o_1, a_2), dot(n_1, a_2));

theta = atan2(sqrt(dot(n_1, a_2)^2 + dot(o_1, a_2)^2), dot(a_1, a_2));

S_phi = -sin(psi)*cos(psi)*(1-cos(theta))*dot(n_1, n_2) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*dot(o_1, n_2) ...
       - sin(psi)*sin(theta)*dot(a_1, n_2);
   
C_phi = -sin(psi)*cos(psi)*(1-cos(theta))*dot(n_1, o_2) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*dot(o_1, o_2) ...
       - sin(psi)*sin(theta)*dot(a_1, o_2);
   
phi = atan2(S_phi, C_phi);