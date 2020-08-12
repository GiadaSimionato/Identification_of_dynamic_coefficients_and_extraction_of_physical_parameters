% @author Giada Simionato, 1822614.
% Function that checks whether a set of dynamic parameters is feasible

function decision = is_feasible(p)

r1 = 0.04;    % radius first link [m]
r2 = 0.035;   % radius second link [m]
r3 = 0.03;    % radius third link [m]
l1 = 0.3;     % first link length [m]
l2 = 0.22;    % second link length [m]
l3 = 0.15;    % third link length [m]

decision = true;

J1 = [p(13), p(14), p(15); p(14), p(16), p(17); p(15), p(17), p(18)];
J2 = [p(19), p(20), p(21); p(20), p(22), p(23); p(21), p(23), p(24)];
J3 = [p(25), p(26), p(27); p(26), p(28), p(29); p(27), p(29), p(30)];

if not(and(and(p(1)>0, p(2)>0), p(3)>0))
   disp('negative masses');
   decision = false;
end
if trace(J1)/2 - max(eig(J1))<=0
    disp('first inertia tensor not positive definite');
    decision = false;
end
if trace(J2)/2 - max(eig(J2))<=0
    disp('second inertia tensor not positive definite');
    decision = false;
end
if trace(J3)/2 - max(eig(J3))<=0
    disp('third inertia tensor not positive definite');
    decision = false;
end
if not(and(p(4)<=r1, p(4)>=-r1))
    disp('c1x out of bounds');
    decision = false;
end
if not(and(p(5)<=0, p(5)>=-l1))
    disp('c1y out of bounds');
    decision = false;
end
if not(and(p(6)<=r1, p(6)>=-r1))
    disp('c1z out of bounds');
    decision = false;
end
if not(and(p(7)<=0, p(7)>=-l2))
    disp('c2x out of bounds');
    decision = false;
end
if not(and(p(8)<=r2, p(8)>=-r2))
    disp('c2y out of bounds');
    decision = false;
end
if not(and(p(9)<=r2, p(9)>=-r2))
    disp('c2z out of bounds');
    decision = false;
end
if not(and(p(10)<=0, p(10)>=-l3))
    disp('c3x out of bounds');
    decision = false;
end
if not(and(p(11)<=r3, p(11)>=-r3))
    disp('c3y out of bounds');
    decision = false;
end
if not(and(p(12)<=r3, p(12)>=-r3))
    disp('c3z out of bounds');
    decision = false;
end
end