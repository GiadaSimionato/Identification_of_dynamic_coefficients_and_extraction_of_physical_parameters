
function tau = get_tau_sym_3R()

disp('Starting algorithm...');

%% Dynamic parameters initialization

syms q1 q2 q3 dq1 dq2 dq3 real      % init sym joint positions and velocities (assumption: real)
syms ddq1 ddq2 ddq3 real   % init sym joint accelerations (assump: real)
syms c1x c2x c1y c2y c1z c2z c3x c3y c3z real       % init sym coords CoMi (assumption: real)
syms J1xx J1xy J1xz J1yy J1yz J1zz J2xx J2xy J2xz J2yy J2yz J2zz J3xx J3xy J3xz J3yy J3yz J3zz real % init inertia tensors elems wrt RFi (assump: real)
syms m1 m2 m3 real        % init sym masses (assump: real)
syms g0 real              % init gravity (assump: real)

%% Data structures initialization

q = [q1; q2; q3];         % vector of joint positions
dq = [dq1; dq2; dq3];     % vector of joint velocities
ddq = [ddq1; ddq2; ddq3]; % vector of joint accelerations
g_vect = [0; 0; -g0];     % gravity vector
r_1_c1 = [c1x; c1y; c1z]; % vector position CoM 1
r_2_c2 = [c2x; c2y; c2z]; % vector position CoM 2
r_3_c3 = [c3x; c3y; c3z]; % vector position CoM 2
r_i_ci = [r_1_c1, r_2_c2, r_3_c3];  % vector positions CoM i
masses = [m1 m2 m3];      % vector masses
J1 = [J1xx, J1xy, J1xz; J1xy, J1yy, J1yz; J1xz, J1yz, J1zz];  % stacked inertia matrix for link 1
J2 = [J2xx, J2xy, J2xz; J2xy, J2yy, J2yz; J2xz, J2yz, J2zz];  % stacked inertia matrix for link 2
J3 = [J3xx, J3xy, J3xz; J3xy, J3yy, J3yz; J3xz, J3yz, J3zz];  % stacked inertia matrix for link 2
J = {J1,J2, J3};       % stacked interia tensors
sigma = zeros(1,3);   % all revolute joints

%% Denavit-Hartenberg parameters initialization

syms a2 a3 real % DH parameters 

%% Tranformation matrices computation

disp('Transformation matrices computation...');

A1 = get_A_matrix(0, pi/2, 0, q1); % get transf. matrix
A2 = get_A_matrix(a2, 0, 0, q2);
A3 = get_A_matrix(a3, 0, 0, q3);
A = {A1, A2, A3};                          % stacked transformation matrices

%% Kinetic energy and inertia tensor computation

disp('Kinetic energy computation:');
T = get_kinetic_energy(dq, A, sigma, r_i_ci, masses, J); % get sym. kinetic energy
disp('Inertia matrix computation...');
M_sym = get_inertia_tensor(T, dq);  % get sym form of inertia tensor

%% Potential energy and gravity vector computation

disp('Potential energy computation:');
U = get_potential_energy(A,r_i_ci,masses,g_vect);   % get sym. potential energy
disp('Gravity vector computation...');
g_sym = get_gravity_vector(U, q);    % get sym for gravity vector

%% S(q, dq) computation
disp('S matrix computation...');
S_sym = get_S(q,dq, M_sym);  % get sym form S(q, dq)

tau = M_sym*ddq + S_sym*dq + g_sym;

disp('Algorithm completed.');
end
