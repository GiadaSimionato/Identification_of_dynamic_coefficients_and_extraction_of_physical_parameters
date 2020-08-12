% Function that returns the symbolic form of the potential energy of the
% manipulator.
% [Adapted from the code provided by Claudio Gaz.]

function U = get_potential_energy(A, r_i_ci, masses, g_vect)

n = length(masses); % number of joints
U = 0;   % init potential energy

for i=1:n  % for each joint
    fprintf('Potential energy comp. of the %d joint\n', i);
    A_curr = eye(4,4); % 4x4 identity matrix
    for j=1:i
        A_curr = A_curr*A{j};
    end
    r_curr = [r_i_ci(:,i);1];      % homogen. r_i_ci__i vector
    r0_ci_hom = A_curr*r_curr;
    r0_ci = collect(simplify(r0_ci_hom(1:3,:)));
    m_i = masses(i);
    U_curr = simplify(-m_i*g_vect'*r0_ci);
    
    U = U + U_curr;
end

U = collect(simplify(U));
end