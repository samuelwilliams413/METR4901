%% function [] = get_Integral()
%
%%

function [integral] = get_Integral(func, wrt) 
% W.R.T. > With Respect To

aa = sym(wrt)
integral = subs(int(subs(func,wrt,aa),aa),aa,wrt)

end