% Function that returns the symbolic form of the kinetic energy of the
% manipulator, computed using moving frames algorithm.
% [Adapted from the code provided by Claudio Gaz.]

function T=get_kinetic_energy(dq, A, sigma, r_i_ci, masses, J)

%% Initialization of linear and angular velocities

n = length(dq); % get number of jonts
V_i = sym('V_i', [3 n]);  % velocities origins RFs
W_i = sym('W_i', [3 n]);  % angular velocities of links

T = 0;          % init. kinetic energy

%% Energy computation

for i=1:n    % for each joint
    
    fprintf('Kinetic energy comp. of the %d joint\n', i);
    if i==1  % set initial conditions for recursion
        v_im1_im1 = [0;0;0];
        w_im1_im1 = [0;0;0];
    else     % get velocities of the previous link wrt previous frame
        v_im1_im1 = V_i(:,i-1);  
        w_im1_im1 = W_i(:,i-1);
    end
    
    R_im1_i = A{i}(1:3,1:3);  % get rotation matrix R from transf. matrix A
    r_im1_im1_i = A{i}(1:3,4);
    
    w_im1_i = w_im1_im1 + (1-sigma(i))*dq(i)*[0;0;1];
    w_i_i = simplify (R_im1_i' * w_im1_i);
    v_i_i = simplify (R_im1_i' * (v_im1_im1 + sigma(i)*dq(i)*[0;0;1] + cross(w_im1_i,r_im1_im1_i)));
    
    m_i = masses(i);
    MS_i = m_i*r_i_ci(:,i);
    
    T = T + collect(simplify( 0.5 * v_i_i' * masses(i) * v_i_i + 0.5 * w_i_i' * J{i} * w_i_i + MS_i'*cross(v_i_i,w_i_i))); 

    V_i(:,i) = v_i_i;  % linear and angular velocities update
    W_i(:,i) = w_i_i;
end
T = collect(simplify(T));
end