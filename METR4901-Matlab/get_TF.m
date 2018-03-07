%% function [] = get_TF(Transfer_Functions)
%
%%
function [] = get_TF(Transfer_Functions)
clc
verbose = 1;
syms s

signpost(verbose,'Start: get_TF()')
%% Initialise variables
signpost(verbose,'Initialise variables')

%% Real Boy
signpost(verbose,'Create Real Values')

%% Collect s
signpost(verbose,'Collect s')

for i = 1:3
    CO(i,1) = rhs(collect(Transfer_Functions(i,1), s));
end

%% Find coefficients of polynomial
signpost(verbose,'Find coefficients of polynomial')

for i = 1:3
    cn = 0;
    cd = 0;
    if (CO(i,1) ~= 0)
        [n, d] = numden(CO(i,1));
        [cn, tn] = coeffs(n, s, 'all');
        [cd, td] = coeffs(d, s, 'all');
        p(cn)
        p(cd)
    end
    eq_n(i,:) = cn
    eq_d(i,:) = cd
end

%% Create tunable real parameters
% Create tunable real parameters with an initial value of 1.

% 1
l1 = realp('l1',1);
L1 = realp('L1',1);
a1 = realp('a1',1);
da1 = realp('da1',1);
dda1 = realp('dda1',1);
Ixx1 = realp('Ixx1',1);
Iyy1 = realp('Iyy1',1);
Izz1 = realp('Izz1',1);
m1 = realp('m1',1);

% 2
l2 = realp('l2',2);
L2 = realp('L2',2);
a2 = realp('a2',2);
da2 = realp('da2',2);
dda2 = realp('dda2',2);
Ixx2 = realp('Ixx2',2);
Iyy2 = realp('Iyy2',2);
Izz2 = realp('Izz2',2);
m2 = realp('m2',2);

% 3
l3 = realp('l3',3);
L3 = realp('L3',3);
a3 = realp('a3',3);
da3 = realp('da3',3);
dda3 = realp('dda3',3);
Ixx3 = realp('Ixx3',3);
Iyy3 = realp('Iyy3',3);
Izz3 = realp('Izz3',3);
m3 = realp('m3',3);

%% Creating Transfer Functions from polynomial
signpost(verbose,'Creating Transfer Functions from polynomial')
for i = 1:3
    eq_n(i,:)
    
    q = eq_d(i,1)
    class(q)
    subs(q, Izz1, 8)
end


%% Tidy Up

signpost(verbose,'Done: get_TF()')

p(CO)
p(eq_n)
p(eq_d)
p(H)
end