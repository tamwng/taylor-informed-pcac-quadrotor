function [u_next,info] = solve_pcac_qp(A, B, d, x0, u0, utrim, reference, p)
%SOLVE_PCAC_QP Jacobian-frozen PCAC QP with the one-step input delay.

n = 12;
m = 4;
N = p.N;
M = N - 1;
if N < 2
    error('solve_pcac_qp:Horizon','The PCAC horizon must satisfy N >= 2.');
end
if ~isequal(size(reference),[n,N])
    error('solve_pcac_qp:Reference','reference must be 12-by-N.');
end

nu = m*M;
nc = size(p.Hx,1);
ne = nc*N;
delta0 = u0(:) - utrim(:);

T = zeros(n*N,nu);
g = zeros(n*N,1);
Ti = zeros(n,nu);
gi = A*x0(:) + B*delta0 + d(:);
T(1:n,:) = Ti;
g(1:n) = gi;
for stage = 2:N
    Ti = A*Ti;
    columns = (stage-2)*m + (1:m);
    Ti(:,columns) = Ti(:,columns) + B;
    gi = A*gi + d(:);
    rows = (stage-1)*n + (1:n);
    T(rows,:) = Ti;
    g(rows) = gi;
end

Qbar = kron(eye(N),p.Q);
last_state = (N-1)*n + (1:n);
Qbar(last_state,last_state) = p.Qf;
Rbar = kron(eye(M),p.R);

Dinc = zeros(nu,nu);
for stage = 1:M
    rows = (stage-1)*m + (1:m);
    Dinc(rows,rows) = eye(m);
    if stage > 1
        previous = (stage-2)*m + (1:m);
        Dinc(rows,previous) = -eye(m);
    end
end
increment_offset = zeros(nu,1);
increment_offset(1:m) = -delta0;

r = reference(:);
Huu = T.'*Qbar*T + Dinc.'*Rbar*Dinc;
fu = T.'*Qbar*(g-r) + Dinc.'*Rbar*increment_offset;

if nc > 0
    Sbar = kron(eye(N),p.S);
    H = blkdiag(Huu,Sbar);
    f = [fu; zeros(ne,1)];

    Hxbar = kron(eye(N),p.Hx);
    hxbar = repmat(p.hx,N,1);
    Astate = [Hxbar*T, -eye(ne)];
    bstate = hxbar - Hxbar*g;
else
    H = Huu;
    f = fu;
    Astate = zeros(0,nu);
    bstate = zeros(0,1);
end

Aincrement = [Dinc, zeros(nu,ne); -Dinc, zeros(nu,ne)];
bincrement = [repmat(p.du_max,M,1) - increment_offset; ...
             -repmat(p.du_min,M,1) + increment_offset];
Aineq = [Astate; Aincrement];
bineq = [bstate; bincrement];

lb_u = repmat(p.u_min-utrim(:),M,1);
ub_u = repmat(p.u_max-utrim(:),M,1);
lb = [lb_u; zeros(ne,1)];
ub = [ub_u; inf(ne,1)];

U0 = repmat(delta0,M,1);
if nc > 0
    slack0 = max(kron(eye(N),p.Hx)*(T*U0+g)-repmat(p.hx,N,1),0);
    w0 = [U0; slack0 + 1.0e-10];
else
    w0 = U0;
end

H = 0.5*(H+H.') + p.qp.hessian_regularization*eye(size(H));
options = optimoptions('quadprog', ...
    'Algorithm',p.qp.algorithm, ...
    'Display','off', ...
    'MaxIterations',p.qp.max_iterations, ...
    'ConstraintTolerance',p.qp.constraint_tolerance);

solve_timer = tic;
[w,objective,exitflag,output] = quadprog(H,f,Aineq,bineq,[],[],lb,ub,w0,options);
solve_time = toc(solve_timer);

success = exitflag > 0 && ~isempty(w) && all(isfinite(w));
if success
    delta1 = w(1:m);
    u_candidate = utrim(:) + delta1;
else
    w = w0;
    objective = nan;
    u_candidate = u0(:);
end

physical_lower = max(p.u_min,u0(:)+p.du_min);
physical_upper = min(p.u_max,u0(:)+p.du_max);
u_next = min(max(u_candidate,physical_lower),physical_upper);
projection_norm = norm(u_next-u_candidate,inf);

if isempty(Aineq)
    inequality_residual = -inf;
else
    inequality_residual = max(Aineq*w-bineq);
end
bound_residual = max([lb-w; w-ub]);

info.success = success;
info.exitflag = exitflag;
info.objective = objective;
info.solve_time = solve_time;
info.iterations = output.iterations;
info.constraint_residual = max(inequality_residual,bound_residual);
info.safety_projection_norm = projection_norm;
info.delta_sequence = reshape(w(1:nu),m,M);
info.predicted_state = reshape(T*w(1:nu)+g,n,N);
if nc > 0
    info.slack = reshape(w(nu+1:end),nc,N);
    info.max_slack = max(info.slack(:));
else
    info.slack = zeros(0,N);
    info.max_slack = 0;
end
end
