% @author Giada Simionato, 1822614.
% Function that checks whether a set of dynamic parameters is feasible

function decision = is_feasible(p)

decision = true;

[LB, UB] = read_bounds('../Data/bounds/bounds_7R_normal.csv');

J1 = [p(29), p(30), p(31); p(30), p(32), p(33); p(31), p(33), p(34)];
J2 = [p(35), p(36), p(37); p(36), p(38), p(39); p(37), p(39), p(40)];
J3 = [p(41), p(42), p(43); p(42), p(44), p(45); p(43), p(45), p(46)];
J4 = [p(47), p(48), p(49); p(48), p(50), p(51); p(49), p(51), p(52)];
J5 = [p(53), p(54), p(55); p(54), p(56), p(57); p(55), p(57), p(58)];
J6 = [p(59), p(60), p(61); p(60), p(62), p(63); p(61), p(63), p(64)];
J7 = [p(65), p(66), p(67); p(66), p(68), p(69); p(67), p(69), p(70)];

B = any([(p(1)>0),(p(2)>0),(p(3)>0),(p(4)>0),(p(5)>0),(p(6)>0),(p(7)>0)]);
for i=1:length(B)
    if B(i)==0
       decision = false;
       fprintf('Negative mass of link %d /n', i);
    end
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
if trace(J4)/2 - max(eig(J4))<=0
    disp('fourth inertia tensor not positive definite');
    decision = false;
end
if trace(J5)/2 - max(eig(J5))<=0
    disp('fifth inertia tensor not positive definite');
    decision = false;
end
if trace(J6)/2 - max(eig(J6))<=0
    disp('sixth inertia tensor not positive definite');
    decision = false;
end
if trace(J7)/2 - max(eig(J7))<=0
    disp('seventh inertia tensor not positive definite');
    decision = false;
end
% Link 1

if or(p(8)<LB(8), p(8)>UB(8))
    disp('c1x out of bounds');
    decision = false;
end
if or(p(9)<LB(9), p(9)>UB(9))
    disp('c1y out of bounds');
    decision = false;
end
if or(p(10)<LB(10), p(10)>UB(10))
    disp('c1z out of bounds');
    decision = false;
end

% Link 2

if or(p(11)<LB(11), p(11)>UB(11))
    disp('c2x out of bounds');
    decision = false;
end
if or(p(12)<LB(12), p(12)>UB(12))
    disp('c2y out of bounds');
    decision = false;
end
if or(p(13)<LB(13), p(13)>UB(13))
    disp('c2z out of bounds');
    decision = false;
end

% Link 3

if or(p(14)<LB(14), p(14)>UB(14))
    disp('c3x out of bounds');
    decision = false;
end
if or(p(15)<LB(15), p(15)>UB(15))
    disp('c3y out of bounds');
    decision = false;
end
if or(p(16)<LB(16), p(16)>UB(16))
    disp('c3z out of bounds');
    decision = false;
end

% Link 4

if or(p(17)<LB(17), p(17)>UB(17))
    disp('c4x out of bounds');
    decision = false;
end
if or(p(18)<LB(18), p(18)>UB(18))
    disp('c4y out of bounds');
    decision = false;
end
if or(p(19)<LB(19), p(19)>UB(19))
    disp('c4z out of bounds');
    decision = false;
end

% Link 5

if or(p(20)<LB(20), p(20)>UB(20))
    disp('c5x out of bounds');
    decision = false;
end
if or(p(21)<LB(21), p(21)>UB(21))
    disp('c5y out of bounds');
    decision = false;
end
if or(p(22)<LB(22), p(22)>UB(22))
    disp('c5z out of bounds');
    decision = false;
end


% Link 6 

if or(p(23)<LB(23), p(23)>UB(23))
    disp('c6x out of bounds');
    decision = false;
end
if or(p(24)<LB(24), p(24)>UB(24))
    disp('c6y out of bounds');
    decision = false;
end
if or(p(25)<LB(25), p(25)>UB(25))
    disp('c6z out of bounds');
    decision = false;
end

% Link 7 

if or(p(26)<LB(26), p(26)>UB(26))
    disp('c7x out of bounds');
    decision = false;
end
if or(p(27)<LB(27), p(27)>UB(27))
    disp('c7y out of bounds');
    decision = false;
end
if or(p(28)<LB(28), p(28)>UB(28))
    disp('c7z out of bounds');
    decision = false;
end

end