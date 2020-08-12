
function [tau , joint_forcemomentum] = core_NE_wrench(q,dq,ddq,...
    g,...
    masses,inertias,...
    R_matrices,r_im1_i_vectors,r_i_Ci_vectors,...
    a0,w0,w0dot,F_last,M_last)

num_of_links = length(q);

W = zeros(3,num_of_links+1); % angular velocity
Wdot = zeros(3,num_of_links+1); % angular acceleration
A = zeros(3,num_of_links+1); % linear acceleration of frame
Ac = zeros(3,num_of_links+1); % linear acceleration of the center of mass
F = zeros(3,num_of_links+1); % forces
M = zeros(3,num_of_links+1); % momenta
tau = zeros(num_of_links,1); % joint torques
joint_forcemomentum = zeros(6,num_of_links); % forces and momenta to be returned

if isa(q,'sym')
    W = sym(W);
    Wdot = sym(Wdot);
    A = sym(A);
    Ac = sym(Ac);
    F = sym(F);
    M = sym(M);
    tau = sym(tau);
    joint_forcemomentum = sym(joint_forcemomentum);
end

% initialize the algorithm
A(:,1) = a0 - g;
W(:,1) = w0;
Wdot(:,1) = w0dot;
F(:,num_of_links+1) = F_last;
M(:,num_of_links+1) = M_last;


% forward recursion
for i=1:num_of_links
    s = sprintf('Forward recursion for link %d',i);
    disp(s);
    Ri = R_matrices{i};
    r_im1_i = r_im1_i_vectors(:,i);
    r_i_Ci = r_i_Ci_vectors(:,i);
    
    w_pre = W(:,i);
    wdot_pre = Wdot(:,i);
    a_pre = A(:,i);
    
    wi = Ri'*(w_pre+[0;0;dq(i)]);
    wi_dot = Ri'*(wdot_pre + [0;0;ddq(i)] + cross(dq(i)*w_pre,[0;0;1]));
    ai = Ri'*a_pre + cross(wi_dot,r_im1_i) + cross(wi,cross(wi,r_im1_i));
    aci = ai + cross(wi_dot,r_i_Ci) + cross(wi,cross(wi,r_i_Ci));
    
    W(:,i+1) = wi;
    Wdot(:,i+1) = wi_dot;
    A(:,i+1) = ai;
    Ac(:,i+1) = aci;
    
end

% backward recursion
for i=num_of_links:-1:1
    s = sprintf('Backward recursion for link %d',i);
    disp(s);
    mi = masses(i);
    Ii = inertias{i};
    R_ip1 = R_matrices{i+1};
    
    r_im1_i = r_im1_i_vectors(:,i);
    r_i_Ci = r_i_Ci_vectors(:,i);
    
    wi = W(:,i+1);
    wi_dot = Wdot(:,i+1);
    aci = Ac(:,i+1);
    f_post = F(:,i+1);
    mom_post = M(:,i+1);
    
    fi = R_ip1*f_post + mi*aci;
    momi = R_ip1*mom_post - cross(fi,r_im1_i + r_i_Ci) ...
        + cross(R_ip1*f_post,r_i_Ci) + Ii*wi_dot + cross(wi,Ii*wi);
    
    F(:,i) = fi;
    M(:,i) = momi;

end

for i=1:num_of_links
    s = sprintf('z-axis projection for link %d',i);
    disp(s);
    Ri = R_matrices{i};
    z_im1 = Ri'*[0;0;1];
    tau(i,:) = M(:,i)'*z_im1;
    joint_forcemomentum(:,i) = [F(:,i);M(:,i)];
end

end