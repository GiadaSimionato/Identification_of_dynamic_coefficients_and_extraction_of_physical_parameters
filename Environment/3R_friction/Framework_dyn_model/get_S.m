% Function that returns the symbolic form of the S(q, dq) matrix that accounts for 
% the Coriolis and centifugal terms in the manipulator.
% [Adapted from the code provided by Claudio Gaz.]

function S_sym = get_S(q,dq,M_sym)
    
    n = length(q);  % num_joints
    S_sym = sym('S_sym',n);  % create nxn symbolic matrix
    for k=1:n
        dMkdq = simplify(jacobian(M_sym(:,k),q));
        dMdqk = simplify(diff(M_sym,q(k)));
        S_sym(k,:) = dq' * 1/2 * (dMkdq + dMkdq' - dMdqk);
    end
    S_sym = collect(simplify(S_sym));
end