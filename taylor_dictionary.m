function dict = taylor_dictionary(D)
%TAYLOR_DICTIONARY Row-wise sparse dictionaries from Tables I and II.

if ~ismember(D,[1 2 3])
    error('taylor_dictionary:Degree','D must be 1, 2, or 3.');
end

nz = 16;
state_names = {'p1','p2','p3','psi','phi','theta', ...
               'v1','v2','v3','omega1','omega2','omega3'};

baseE = cell(12,1); baseN = cell(12,1);
add2E = cell(12,1); add2N = cell(12,1);
add3E = cell(12,1); add3N = cell(12,1);

baseE{1} = [mono(nz,1,1); mono(nz,7,1); mono(nz,6,1); mono(nz,11,1); mono(nz,15,1)];
baseN{1} = {'p1','v1','theta','omega2','tau2'};
baseE{2} = [mono(nz,2,1); mono(nz,8,1); mono(nz,5,1); mono(nz,10,1); mono(nz,14,1)];
baseN{2} = {'p2','v2','phi','omega1','tau1'};
baseE{3} = [mono(nz,3,1); mono(nz,9,1); mono(nz,[],[]); mono(nz,13,1)];
baseN{3} = {'p3','v3','1','ftilde'};
baseE{4} = [mono(nz,4,1); mono(nz,12,1); mono(nz,16,1)];
baseN{4} = {'psi','omega3','tau3'};
baseE{5} = [mono(nz,5,1); mono(nz,10,1); mono(nz,14,1)];
baseN{5} = {'phi','omega1','tau1'};
baseE{6} = [mono(nz,6,1); mono(nz,11,1); mono(nz,15,1)];
baseN{6} = {'theta','omega2','tau2'};
baseE{7} = [mono(nz,7,1); mono(nz,6,1); mono(nz,11,1); mono(nz,15,1)];
baseN{7} = {'v1','theta','omega2','tau2'};
baseE{8} = [mono(nz,8,1); mono(nz,5,1); mono(nz,10,1); mono(nz,14,1)];
baseN{8} = {'v2','phi','omega1','tau1'};
baseE{9} = [mono(nz,9,1); mono(nz,[],[]); mono(nz,13,1)];
baseN{9} = {'v3','1','ftilde'};
baseE{10} = [mono(nz,10,1); mono(nz,14,1)];
baseN{10} = {'omega1','tau1'};
baseE{11} = [mono(nz,11,1); mono(nz,15,1)];
baseN{11} = {'omega2','tau2'};
baseE{12} = [mono(nz,12,1); mono(nz,16,1)];
baseN{12} = {'omega3','tau3'};

add2E{1} = [mono(nz,[5 4],[1 1]); mono(nz,[13 6],[1 1])];
add2N{1} = {'phi*psi','ftilde*theta'};
add2E{2} = [mono(nz,[4 6],[1 1]); mono(nz,[13 5],[1 1])];
add2N{2} = {'psi*theta','ftilde*phi'};
add2E{3} = [mono(nz,5,2); mono(nz,6,2)];
add2N{3} = {'phi^2','theta^2'};
add2E{4} = mono(nz,[6 10],[1 1]);
add2N{4} = {'theta*omega1'};
add2E{5} = mono(nz,[6 12],[1 1]);
add2N{5} = {'theta*omega3'};
add2E{6} = mono(nz,[5 12],[1 1]);
add2N{6} = {'phi*omega3'};
add2E{7} = add2E{1}; add2N{7} = add2N{1};
add2E{8} = add2E{2}; add2N{8} = add2N{2};
add2E{9} = add2E{3}; add2N{9} = add2N{3};
add2E{10} = mono(nz,[11 12],[1 1]);
add2N{10} = {'omega2*omega3'};
add2E{11} = mono(nz,[12 10],[1 1]);
add2N{11} = {'omega3*omega1'};
add2E{12} = mono(nz,[10 11],[1 1]);
add2N{12} = {'omega1*omega2'};

add3E{1} = [mono(nz,6,3); mono(nz,[4 6],[2 1]); mono(nz,[13 5 4],[1 1 1])];
add3N{1} = {'theta^3','psi^2*theta','ftilde*phi*psi'};
add3E{2} = [mono(nz,5,3); mono(nz,[5 4],[1 2]); mono(nz,[5 6],[1 2]); mono(nz,[13 4 6],[1 1 1])];
add3N{2} = {'phi^3','phi*psi^2','phi*theta^2','ftilde*psi*theta'};
add3E{3} = [mono(nz,[13 5],[1 2]); mono(nz,[13 6],[1 2])];
add3N{3} = {'ftilde*phi^2','ftilde*theta^2'};
add3E{4} = [mono(nz,[5 12],[2 1]); mono(nz,[6 12],[2 1])];
add3N{4} = {'phi^2*omega3','theta^2*omega3'};
add3E{5} = mono(nz,[6 10],[2 1]);
add3N{5} = {'theta^2*omega1'};
add3E{6} = mono(nz,[5 6 10],[1 1 1]);
add3N{6} = {'phi*theta*omega1'};
add3E{7} = add3E{1}; add3N{7} = add3N{1};
add3E{8} = add3E{2}; add3N{8} = add3N{2};
add3E{9} = add3E{3}; add3N{9} = add3N{3};
for q = 10:12
    add3E{q} = zeros(0,nz);
    add3N{q} = cell(0,1);
end

exponents = cell(12,1);
names = cell(12,1);
counts = zeros(12,1);
for q = 1:12
    E = baseE{q};
    N = baseN{q}(:);
    if D >= 2
        E = [E; add2E{q}]; %#ok<AGROW>
        N = [N; add2N{q}(:)]; %#ok<AGROW>
    end
    if D >= 3
        E = [E; add3E{q}]; %#ok<AGROW>
        N = [N; add3N{q}(:)]; %#ok<AGROW>
    end
    [E,N] = unique_within_row(E,N);
    exponents{q} = E;
    names{q} = N;
    counts(q) = size(E,1);
end

dict.D = D;
dict.z_dimension = nz;
dict.state_names = state_names;
dict.exponents = exponents;
dict.names = names;
dict.counts = counts;
dict.total_coefficients = sum(counts);

expected = [40 58 80];
if dict.total_coefficients ~= expected(D)
    error('taylor_dictionary:Count', ...
        'Dictionary D=%d has %d coefficients; expected %d.', ...
        D, dict.total_coefficients, expected(D));
end
end

function e = mono(nz, index, power)
e = zeros(1,nz);
if isempty(index)
    return
end
index = index(:);
power = power(:);
if numel(power) == 1 && numel(index) > 1
    power = repmat(power,numel(index),1);
end
for k = 1:numel(index)
    e(index(k)) = e(index(k)) + power(k);
end
end

function [Eout,Nout] = unique_within_row(E,N)
keep = true(size(E,1),1);
for i = 2:size(E,1)
    if any(all(E(1:i-1,:) == E(i,:),2))
        keep(i) = false;
    end
end
Eout = E(keep,:);
Nout = N(keep);
end
