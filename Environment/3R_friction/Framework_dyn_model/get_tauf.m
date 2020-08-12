% @author Giada Simionato, 1822614.
% Function that returns the symbolic form of the friction torque term in a 3R manipulator.

function tauf = get_tauf(dq)

syms fv1 fv2 fv3 real     % viscous friction coefficients
syms fc1 fc2 fc3 real     % Coulomb friction coefficients
syms fo1 fo2 fo3 real     % Coulomb friction offsets

n_joints = size(dq, 1);
sign_dq = sign(dq);
fv = [fv1, fv2, fv3];
fc = [fc1, fc2, fc3];
fo = [fo1, fo2, fo3];

for i=1:n_joints
    tauf(i) = fv(i)*dq(i) + fo(i)+ fc(i)*sign_dq(i);
end

end