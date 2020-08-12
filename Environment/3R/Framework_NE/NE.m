% @author Giada Simioanto, 1822614.
% Script that returns the symbolic form of the dynamic model of a 3R
% manipulator using the Newton-Euler algorithm.
% [Adapted from the code by Claudio Gaz.]

function [tau , joint_forcemomentum] = NE()

syms g0...
    q1 q2 q3 ...
    dq1 dq2 dq3 ...
    ddq1 ddq2 ddq3...
    m1 m2 m3...
    J1xx J1xy J1xz J1yy J1yz J1zz ...
    J2xx J2xy J2xz J2yy J2yz J2zz ...
    J3xx J3xy J3xz J3yy J3yz J3zz ...
    c1x c1y c1z c2x c2y c2z c3x c3y c3z ...
    a2 a3 ...
    real

q = [q1 ; q2 ; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];

w0 = sym([0;0;0]);
w0dot = sym([0;0;0]);
a0 = sym([0;0;0]);

F_last = sym([0;0;0]);
M_last = sym([0;0;0]);

g = [0;0;-g0];

masses = [m1,m2,m3];
r_1_C1 = [c1x;c1y;c1z];
r_2_C2 = [c2x;c2y;c2z];
r_3_C3 = [c3x;c3y;c3z];

r_i_Ci_vectors = [r_1_C1,r_2_C2,r_3_C3];

J1 = [J1xx,J1xy,J1xz;J1xy,J1yy,J1yz;J1xz,J1yz,J1zz];
J2 = [J2xx,J2xy,J2xz;J2xy,J2yy,J2yz;J2xz,J2yz,J2zz];
J3 = [J3xx,J3xy,J3xz;J3xy,J3yy,J3yz;J3xz,J3yz,J3zz];

I1 = J1-m1*skew_symmetric(r_1_C1)'*skew_symmetric(r_1_C1);
I2 = J2-m2*skew_symmetric(r_2_C2)'*skew_symmetric(r_2_C2);
I3 = J3-m3*skew_symmetric(r_3_C3)'*skew_symmetric(r_3_C3);

inertias = {I1,I2,I3};

R1=R(q(1),pi/2);
R2=R(q(2),0);
R3=R(q(3),0);

R_matrices{1} = R1;
R_matrices{2} = R2;
R_matrices{3} = R3;
R_matrices{4} = eye(3);

r_0_1 = p(0,0,pi/2);
r_1_2 = p(0,a2,0);
r_2_3 = p(0,a3,0);
    
r_im1_i_vectors = [r_0_1,r_1_2,r_2_3];

[tau , joint_forcemomentum] = core_NE_wrench(q, dq, ddq, g, masses,inertias, R_matrices, r_im1_i_vectors, r_i_Ci_vectors, a0,w0,w0dot,F_last,M_last);
end