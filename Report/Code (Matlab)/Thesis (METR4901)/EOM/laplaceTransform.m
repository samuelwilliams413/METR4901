%% function [] = laplaceTransform(Torque)
%
%%

function [L] = laplaceTransform(Torque)

N = size(Torque);
N = N(1);

verbose = 1;

signpost(verbose,'Start: laplaceTransform()')

%% Initialise variables
signpost(verbose,'Variable init')

syms s
syms a1 da1 dda1
syms a2 da2 dda2
syms a3 da3 dda3

a_list   = [a1,     a2,     a3];
da_list  = [da1,    da2,    da3];
dda_list = [dda1,   dda2,   dda3];


for index = 1:N
    %% Initialise variables
    signpost(verbose,'Variable init Loop')
    
    a   =   a_list(index);
    da  =  da_list(index);
    dda = dda_list(index);
    
    T  = Torque(index,1);
    eq = Torque(index,2);
    str = ['T',num2str(index),' == ' char(eq)];
    
    L = simplify(laplace(eq, a, s));
    L = simplify(subs(L, da, (1/s)));
    L = simplify(subs(L, dda, 1));
    L = simplify(L*(s^2));
    p(str);
    p(L);
    
end
signpost(verbose,'Done: laplaceTransform()')

end