% Function that returns the symbolic form of the gravity vector g(q) of the
% manipulator.
% [Adapted from the code provided by Claudio Gaz.]

function g_sym = get_gravity_vector(U, q)
    g_sym = collect(simplify(jacobian(U,q)'));
end