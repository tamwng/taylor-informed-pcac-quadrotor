function rls = rowwise_rls(mode, rls, dict, z, prediction_error, beta, p)
%ROWWISE_RLS Twelve independent scalar-output RLS estimators.

switch lower(mode)
    case 'initialize'
        rls = initialize_estimators(dict,p);
    case 'update'
        if isempty(z) || isempty(prediction_error) || isempty(beta)
            error('rowwise_rls:UpdateInputs','z, prediction_error, and beta are required.');
        end
        for q = 1:12
            phi = evaluate_terms(dict.exponents{q},z);
            Pq = rls.P{q};
            Lq = beta*Pq;
            Lphi = Lq*phi;
            denominator = 1 + phi.'*Lphi;
            Pnew = Lq - (Lphi*Lphi.')/denominator;
            Pnew = 0.5*(Pnew + Pnew.');
            if p.rls.jitter > 0
                Pnew = Pnew + p.rls.jitter*eye(size(Pnew));
            end
            if isfinite(p.rls.covariance_eigenvalue_max)
                [V,Dm] = eig(Pnew,'vector');
                Dm = min(max(Dm,p.rls.jitter),p.rls.covariance_eigenvalue_max);
                Pnew = V*diag(Dm)*V.';
                Pnew = 0.5*(Pnew + Pnew.');
            end
            rls.theta{q} = rls.theta{q} + Pnew*phi*prediction_error(q);
            rls.P{q} = Pnew;
        end
        rls.update_count = rls.update_count + 1;
    otherwise
        error('rowwise_rls:Mode','Unknown mode "%s".',mode);
end
end

function rls = initialize_estimators(dict,p)
if ~strcmpi(p.rls.initialization,'nominal-linear-zoh')
    error('rowwise_rls:Initialization','Unsupported initialization method.');
end

Ac = zeros(12,12);
Bc = zeros(12,4);
Ac(1:3,7:9) = eye(3);
Ac(4,12) = 1;
Ac(5,10) = 1;
Ac(6,11) = 1;
Ac(7,6) = p.g;
Ac(8,5) = -p.g;
Bc(9,1) = 1/p.m0;
Bc(10:12,2:4) = p.J\eye(3);

M = expm([Ac Bc; zeros(4,16)]*p.Ts);
Ad = M(1:12,1:12);
Bd = M(1:12,13:16);

rls.theta = cell(12,1);
rls.P = cell(12,1);
for q = 1:12
    E = dict.exponents{q};
    theta = zeros(size(E,1),1);
    for j = 1:size(E,1)
        if sum(E(j,:)) == 1
            index = find(E(j,:) == 1,1);
            if index <= 12
                theta(j) = Ad(q,index);
            else
                theta(j) = Bd(q,index-12);
            end
        end
    end
    rls.theta{q} = theta;
    rls.P{q} = p.rls.P0*eye(size(E,1));
end
rls.update_count = 0;
rls.Ad0 = Ad;
rls.Bd0 = Bd;
end

function phi = evaluate_terms(E,z)
z = z(:);
phi = ones(size(E,1),1);
for i = 1:size(E,1)
    for j = 1:size(E,2)
        if E(i,j) ~= 0
            phi(i) = phi(i)*z(j)^E(i,j);
        end
    end
end
end
