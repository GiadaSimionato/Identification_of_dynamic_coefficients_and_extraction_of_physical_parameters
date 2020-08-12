% @author Giada Simioanto, 1822614.
% Script that returns the symbolic form of the dynamic model of a 7R
% manipulator using the Newton-Euler algorithm.
% [Adapted from the code by Claudio Gaz.]

function tau = NE()

syms g0...
    q1 q2 q3 q4 q5 q6 q7 ...
    dq1 dq2 dq3 dq4 dq5 dq6 dq7 ...
    ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 ...
    m1 m2 m3 m4 m5 m6 m7...
    J1xx J1xy J1xz J1yy J1yz J1zz ...
    J2xx J2xy J2xz J2yy J2yz J2zz ...
    J3xx J3xy J3xz J3yy J3yz J3zz ...
    J4xx J4xy J4xz J4yy J4yz J4zz ...
    J5xx J5xy J5xz J5yy J5yz J5zz ...
    J6xx J6xy J6xz J6yy J6yz J6zz ...
    J7xx J7xy J7xz J7yy J7yz J7zz ...
    c1x c1y c1z c2x c2y c2z c3x c3y c3z ...
    c4x c4y c4z c5x c5y c5z c6x c6y c6z c7x c7y c7z ...
    d1 d2 ...
    real

q = [q1 ; q2 ; q3; q4; q5; q6; q7];
dq = [dq1; dq2; dq3; dq4; dq5; dq6; dq7];
ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6; ddq7];

w0 = sym([0;0;0]);
w0dot = sym([0;0;0]);
a0 = sym([0;0;0]);

F_last = sym([0;0;0]);
M_last = sym([0;0;0]);

g = [0;0;-g0];

masses = [m1,m2,m3,m4,m5,m6,m7];
r_1_C1 = [c1x;c1y;c1z];
r_2_C2 = [c2x;c2y;c2z];
r_3_C3 = [c3x;c3y;c3z];
r_4_C4 = [c4x;c4y;c4z];
r_5_C5 = [c5x;c5y;c5z];
r_6_C6 = [c6x;c6y;c6z];
r_7_C7 = [c7x;c7y;c7z];

r_i_Ci_vectors = [r_1_C1,r_2_C2,r_3_C3, r_4_C4, r_5_C5, r_6_C6, r_7_C7];

J1 = [J1xx,J1xy,J1xz;J1xy,J1yy,J1yz;J1xz,J1yz,J1zz];
J2 = [J2xx,J2xy,J2xz;J2xy,J2yy,J2yz;J2xz,J2yz,J2zz];
J3 = [J3xx,J3xy,J3xz;J3xy,J3yy,J3yz;J3xz,J3yz,J3zz];
J4 = [J4xx,J4xy,J4xz;J4xy,J4yy,J4yz;J4xz,J4yz,J4zz];
J5 = [J5xx,J5xy,J5xz;J5xy,J5yy,J5yz;J5xz,J5yz,J5zz];
J6 = [J6xx,J6xy,J6xz;J6xy,J6yy,J6yz;J6xz,J6yz,J6zz];
J7 = [J7xx,J7xy,J7xz;J7xy,J7yy,J7yz;J7xz,J7yz,J7zz];

I1 = J1-m1*skew_symmetric(r_1_C1)'*skew_symmetric(r_1_C1);
I2 = J2-m2*skew_symmetric(r_2_C2)'*skew_symmetric(r_2_C2);
I3 = J3-m3*skew_symmetric(r_3_C3)'*skew_symmetric(r_3_C3);
I4 = J4-m4*skew_symmetric(r_4_C4)'*skew_symmetric(r_4_C4);
I5 = J5-m5*skew_symmetric(r_5_C5)'*skew_symmetric(r_5_C5);
I6 = J6-m6*skew_symmetric(r_6_C6)'*skew_symmetric(r_6_C6);
I7 = J7-m7*skew_symmetric(r_7_C7)'*skew_symmetric(r_7_C7);

inertias = {I1,I2,I3,I4,I5,I6,I7};

R1=R(q(1),pi/2);
R2=R(q(2),-pi/2);
R3=R(q(3),-pi/2);
R4=R(q(4),pi/2);
R5=R(q(5),pi/2);
R6=R(q(6),-pi/2);
R7=R(q(7),0);

R_matrices{1} = R1;
R_matrices{2} = R2;
R_matrices{3} = R3;
R_matrices{4} = R4;
R_matrices{5} = R5;
R_matrices{6} = R6;
R_matrices{7} = R7;
R_matrices{8} = eye(3);

r_0_1 = p(0,0,pi/2);
r_1_2 = p(0,0,-pi/2);
r_2_3 = p(d1,0,-pi/2);
r_3_4 = p(0,0,pi/2);
r_4_5 = p(d2,0,pi/2);
r_5_6 = p(0,0,-pi/2);
r_6_7 = p(0,0,0);
    
r_im1_i_vectors = [r_0_1,r_1_2,r_2_3,r_3_4,r_4_5,r_5_6,r_6_7];

tau = core_NE(q, dq, ddq, g, masses,inertias, R_matrices, r_im1_i_vectors, r_i_Ci_vectors, a0,w0,w0dot,F_last,M_last);
end