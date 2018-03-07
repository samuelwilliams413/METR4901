%% function [] = imma_real_boy(Transfer_Functions)
%
%%
function [Transfer_Functions] = imma_real_boy(Transfer_Functions)
verbose = 1;

syms g

syms a1 da1 dda1
syms a2 da2 dda2
syms a3 da3 dda3

syms A1 A2 A3
syms t1_a1_T1 t2_a2_T2 t3_a3_T3

syms l1 l2 l3
syms L1 L2 L3

syms m1 m2 m3

syms Ixx1 Ixx2 Ixx3
syms Iyy1 Iyy2 Iyy3
syms Izz1 Izz2 Izz3

syms T1 T2 T3

t = Transfer_Functions;

%% Set Values
g_ = 1;

% 1
l1_ = 1;
L1_ = 1;
a1_ = 1;
da1_ = 1;
dda1_ = 1;
Ixx1_ = 1;
Iyy1_ = 1;
Izz1_ = 1;
m1_ = 1;

% 2
l2_ = 2;
L2_ = 2;
a2_ = 2;
da2_ = 2;
dda2_ = 2;
Ixx2_ = 2;
Iyy2_ = 2;
Izz2_ = 2;
m2_ = 2;

% 3
l3_ = 3;
L3_ = 3;
a3_ = 3;
da3_ = 3;
dda3_ = 3;
Ixx3_ = 3;
Iyy3_ = 3;
Izz3_ = 3;
m3_ = 3;

%% Replace

t = subs(t, g, g_);

%1
t = subs(t, l1, l1_);
t = subs(t, L1, L1_);
t = subs(t, a1, a1_);
t = subs(t, da1, da1_);
t = subs(t, dda1, dda1_);
t = subs(t, Ixx1, Ixx1_);
t = subs(t, Iyy1, Iyy1_);
t = subs(t, Izz1, Izz1_);
t = subs(t, m1, m1_);

%2
t = subs(t, l2, l2_);
t = subs(t, L2, L2_);
t = subs(t, a2, a2_);
t = subs(t, da2, da2_);
t = subs(t, dda2, dda2_);
t = subs(t, Ixx2, Ixx2_);
t = subs(t, Iyy2, Iyy2_);
t = subs(t, Izz2, Izz2_);
t = subs(t, m2, m2_);

%3
t = subs(t, l3, l3_);
t = subs(t, L3, L3_);
t = subs(t, a3, a3_);
t = subs(t, da3, da3_);
t = subs(t, dda3, dda3_);
t = subs(t, Ixx3, Ixx3_);
t = subs(t, Iyy3, Iyy3_);
t = subs(t, Izz3, Izz3_);
t = subs(t, m3, m3_);

%% Tidy Up
Transfer_Functions = t;
signpost(verbose,'Done: imma_real_boy()')
end