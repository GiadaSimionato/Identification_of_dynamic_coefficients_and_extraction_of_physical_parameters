% @author Giada Simionato, 1822614.

function loss = get_loss(x)

global opt_step

% Initialize penalty coefficient

if opt_step == 1
    csi = 0;
else
    csi = 10*(opt_step-1);
end

% Get the current values of the parameters

m1 = x(1);
m2 = x(2);
m3 = x(3);

c1x = x(4);
c1y = x(5);
c1z = x(6);
c2x = x(7);
c2y = x(8);
c2z = x(9);
c3x = x(10);
c3y = x(11);
c3z = x(12);

J1xx = x(13);
J1xy = x(14);
J1xz = x(15);
J1yy = x(16);
J1yz = x(17);
J1zz = x(18);
J2xx = x(19);
J2xy = x(20);
J2xz = x(21);
J2yy = x(22);
J2yz = x(23);
J2zz = x(24);
J3xx = x(25);
J3xy = x(26);
J3xz = x(27);
J3yy = x(28);
J3yz = x(29);
J3zz = x(30);

% Initialize error vector
e = zeros(15,1);

e(1) = (400*J3zz)/9 - (2500*J2zz)/121 + m2 - 0.92358494031217;
e(2) = m3 - (400*J3zz)/9 - 0.799555555555571;
e(3) = (50*J2zz)/11 + m2*c2x + 0.00949090909090944;
e(4) = m2*c2y - 0.000799999999999822;
e(5) = (20*J3xz)/3 - (50*J2xz)/11 + m2*c2z - 0.00296545454545671;
e(6) = (20*J3zz)/3 + m3*c3x + 0.0119333333333355;
e(7) = m3*c3y - 0.00240000000000037;
e(8) = m3*c3z - (20*J3xz)/3 - 0.00108000000000169;
e(9) = J1yy + J2yy + J3yy - J2zz - J3zz - 0.0263100000000015;
e(10) = J2xx - J2yy + J2zz - 0.000497999999997778;
e(11) = J2xy - 0.000126999999999863;
e(12) = J2yz + 3.20000000003651e-06;
e(13) = J3xx - J3yy + J3zz - 0.000560999999999749;
e(14) = J3xy - 0.000252000000000009;
e(15) = J3yz + 1.44000000003447e-05;

loss = e'*e;

end