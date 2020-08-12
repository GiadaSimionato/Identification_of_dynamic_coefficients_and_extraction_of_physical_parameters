% Function that returns the symbolic form of the inertia tensor M(q) of the
% manipulator.
% [Adapted from the code provided by Claudio Gaz.]

function M_sym = get_inertia_tensor(T, dq)
    disp('Computing inertia tensor...');
    M_sym = collect(simplify(hessian(T,dq)));  % double derivatives
end