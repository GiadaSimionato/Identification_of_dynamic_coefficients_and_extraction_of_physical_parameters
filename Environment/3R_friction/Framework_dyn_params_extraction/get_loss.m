% @author: Giada Simionato, 1822614.
% Function that returns the value of the cost function
% [Adapted from the work of Claudio Gaz.]

function loss = get_loss_full(x)

global opt_step
global min_mass
global max_mass

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

fv1 = x(31);
fv2 = x(32);
fv3 = x(33);
fc1 = x(34);
fc2 = x(35);
fc3 = x(36);
fo1 = x(37);
fo2 = x(38);
fo3 = x(39);

% Initialize error vector
e = zeros(24,1);  % 24 is the number of dyn coefficients indentified

e(1) = (400*J3zz)/9 - (2500*J2zz)/121 + m2 - 0.923584940312248;
e(2) = m3 - (400*J3zz)/9 - 0.799555555555588;
e(3) = (50*J2zz)/11 + m2*c2x + 0.00949090909091011;
e(4) = m2*c2y - 0.000799999999999954;
e(5) = (20*J3xz)/3 - (50*J2xz)/11 + m2*c2z - 0.00296545454545497;
e(6) = (20*J3zz)/3 + m3*c3x + 0.0119333333333335;
e(7) = m3*c3y - 0.00239999999999991;
e(8) = m3*c3z - (20*J3xz)/3 - 0.00107999999999892;
e(9) = J1yy + J2yy + J3yy - J2zz - J3zz - 0.0263100000000015;
e(10) = J2xx - J2yy + J2zz - 0.00049800000000081;
e(11) = J2xy - 0.000127000000000932;
e(12) = J2yz + 3.20000000069483e-06;
e(13) = J3xx - J3yy + J3zz - 0.000561000000000707;
e(14) = J3xy - 0.000251999999999558;
e(15) = J3yz + 1.44000000003794e-05;
e(16) = fv1 - 0.0664999999999999;
e(17) = fv2 - 0.1987;
e(18) = fv3 - 0.0398999999999984;
e(19) = fc1 - 0.245;
e(20) = fc2 - 0.1523;
e(21) = fc3 - 0.182700000000001;
e(22) = fo1 + 0.1073;
e(23) = fo2 + 0.156600000000003;
e(24) = fo3 + 0.0685999999999998;
loss = e'*e;

end