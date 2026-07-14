function [xnext,A,B,c,phi_rows] = identified_map_jacobian(rls, dict, z)
%IDENTIFIED_MAP_JACOBIAN Identified sampled map and analytical Jacobian.

z = z(:);
if numel(z) ~= dict.z_dimension
    error('identified_map_jacobian:Dimension','z must have 16 entries.');
end

xnext = zeros(12,1);
Jacobian = zeros(12,dict.z_dimension);
phi_rows = cell(12,1);
for q = 1:12
    [phi,dphi] = evaluate_terms_and_gradient(dict.exponents{q},z);
    theta = rls.theta{q};
    xnext(q) = theta.'*phi;
    Jacobian(q,:) = theta.'*dphi;
    phi_rows{q} = phi;
end

A = Jacobian(:,1:12);
B = Jacobian(:,13:16);
c = xnext - A*z(1:12) - B*z(13:16);
end

function [phi,dphi] = evaluate_terms_and_gradient(E,z)
number_of_terms = size(E,1);
number_of_variables = size(E,2);
phi = ones(number_of_terms,1);
dphi = zeros(number_of_terms,number_of_variables);

for i = 1:number_of_terms
    for j = 1:number_of_variables
        if E(i,j) ~= 0
            phi(i) = phi(i)*z(j)^E(i,j);
        end
    end
    for j = 1:number_of_variables
        exponent = E(i,j);
        if exponent == 0
            continue
        end
        derivative = exponent;
        for ell = 1:number_of_variables
            adjusted_exponent = E(i,ell) - double(ell == j);
            if adjusted_exponent ~= 0
                derivative = derivative*z(ell)^adjusted_exponent;
            end
        end
        dphi(i,j) = derivative;
    end
end
end
