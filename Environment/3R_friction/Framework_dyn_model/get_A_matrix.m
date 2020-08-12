% Function that returns the transformation matrix in symbolic form based on
% the DH parameters provided.
% [Adapted from the code provided by Claudio Gaz.]

function A=get_A_matrix(a, alpha, d, theta)

disp('A matrix computation');
threshold = 1e-10;  % for numerical error

A=[cos(theta)   -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);...
   sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);...
       0            sin(alpha)              cos(alpha)              d;...
       0            0                       0                       1];

%% Adjust numerical errors
   
if isa(A,'sym') % if element of matrix is lower than threshold set it to zero
	for i=1:size(A,1)
		for j=1:size(A,2)
			coeff_ij = eval(coeffs(A(i,j))); % returns evaluation of coefficients of matrix 
			if (abs(coeff_ij) < threshold)
				A(i,j) = 0;
			end
		end
	end
end
end