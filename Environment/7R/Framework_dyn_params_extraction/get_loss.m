
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
m4 = x(4);
m5 = x(5);
m6 = x(6);
m7 = x(7);

c1x = x(8);
c1y = x(9);
c1z = x(10);
c2x = x(11);
c2y = x(12);
c2z = x(13);
c3x = x(14);
c3y = x(15);
c3z = x(16);
c4x = x(17);
c4y = x(18);
c4z = x(19);
c5x = x(20);
c5y = x(21);
c5z = x(22);
c6x = x(23);
c6y = x(24);
c6z = x(25);
c7x = x(26);
c7y = x(27);
c7z = x(28);

J1xx = x(29);
J1xy = x(30);
J1xz = x(31);
J1yy = x(32);
J1yz = x(33);
J1zz = x(34);
J2xx = x(35);
J2xy = x(36);
J2xz = x(37);
J2yy = x(38);
J2yz = x(39);
J2zz = x(40);
J3xx = x(41);
J3xy = x(42);
J3xz = x(43);
J3yy = x(44);
J3yz = x(45);
J3zz = x(46);
J4xx = x(47);
J4xy = x(48);
J4xz = x(49);
J4yy = x(50);
J4yz = x(51);
J4zz = x(52);
J5xx = x(53);
J5xy = x(54);
J5xz = x(55);
J5yy = x(56);
J5yz = x(57);
J5zz = x(58);
J6xx = x(59);
J6xy = x(60);
J6xz = x(61);
J6yy = x(62);
J6yz = x(63);
J6zz = x(64);
J7xx = x(65);
J7xy = x(66);
J7xz = x(67);
J7yy = x(68);
J7yz = x(69);
J7zz = x(70);

% Initialize error vector
e = zeros(43,1);

% Compute error vector, as the difference of current dynamic coeff values
% and previously estimated dyn coeff values, as follows:
% pi(p_k) - pi_hat

e(1) = (25*J2yy)/4 - (10000*J4yy)/1521 + (25*J3zz)/4 - (10000*J5zz)/1521 + m3 + m4 - 5*m3*c3y - (200*m5*c5y)/39 - 5.210244803089981;
e(2) = (10000*J4yy)/1521 + (10000*J5zz)/1521 + m5 + m6 + m7 + (200*m5*c5y)/39 - 3.067140696909943;
e(3) = m2*c2x - 0.003618000000003;
e(4) = m2*c2z - (5*J3zz)/2 - (5*J2yy)/2 + m3*c3y - 0.289045800000019;
e(5) = m3*c3x + 0.003618000000002;
e(6) = m3*c3z + m4*c4y + 2.49800180540660e-15;
e(7) = m4*c4x + 0.003618000000002;
e(8) = m4*c4z - (100*J5zz)/39 - (100*J4yy)/39 - m5*c5y - 0.255008028205123;
e(9) = m5*c5x + 0.001688100000000;
e(10) = m5*c5z - m6*c6y + 0.036298999999999;
e(11) = m6*c6x + 4.143999999995096e-04;
e(12) = m6*c6z + m7*c7z - 0.010375200000001;
e(13) = m7*c7x + 2.192690473634684e-15;
e(14) = m7*c7y - 1.450228825916611e-15;
e(15) = J1yy + J2zz - 0.013773999999998;
e(16) = J2xx - J2yy - J2zz + 0.004866999999994;
e(17)= J2xy - 9.415000000656537e-05;
e(18)= J2xz + 3.145000000033260e-04;
e(19)= J2yz - 0.009746999999999;
e(20)= J3xx - J3zz + J4zz + 0.023225999999997;
e(21)= J3xy - 3.144999999976084e-04;
e(22)= J3xz + 9.557199999665755e-05;
e(23)= J3yy + J4zz - 0.013774200000000;
e(24)= J3yz - 0.002681000000000;
e(25)= J4xx - J4yy - J4zz + 0.006886999999998;
e(26)= J4xy - 9.415000000020240e-05;
e(27)= J4xz - 3.145499999994139e-04;
e(28)= J4yz - 0.009747000000000;
e(29)= J5xx - J5zz + J6zz - 0.005470000000001;
e(30)= J5xy + 1.897999999999275e-04;
e(31)= J5xz + 4.474000000101452e-05;
e(32)= J5yy + J6zz - 0.008415000000002;
e(33)= J5yz + 0.002023000000001;
e(34)= J6xx + J7yy - J6zz - 8.219999999973249e-04;
e(35)= J6xy + 2.463000000174631e-06;
e(36)= J6xz + 2.323000001067110e-06;
e(37)= J6yy + J7yy - 0.004360000000002;
e(38)= J6yz + 7.007999999977810e-05;
e(39)= J7xx - J7yy - 0.002178000000000;
e(40)= J7xy + 2.081668171172169e-16;
e(41)= J7xz + 9.714451465470120e-16;
e(42)= J7yz - 5.689893001203927e-16;
e(43)= J7zz - 1.583999999992258e-04;


loss = e'*e;

end