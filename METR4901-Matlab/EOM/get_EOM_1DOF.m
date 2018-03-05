%% function [] = get_EOm1DOF()
%
%%

function [] = get_EOM_1DOF()

clc
close all
verbose = 1;

signpost(verbose,'Start: get_EOm1DOF()')

%% Initialise variables
signpost(verbose,'Variable init')

%Symbolic Variables
syms g temp
syms a1 da1 dda1 
syms a2 da2 dda2 
syms a3 da3 dda3

syms l1 l2 l3 
syms m1 m2 m3

syms Ixx1 Ixx2 Ixx3
syms Iyy1 Iyy2 Iyy3
syms Izz1 Izz2 Izz3

syms T1 T2 T3

it1 = 3;
it2 = 3;
it3 = 3;

C = [0*temp 0*temp 0*temp; 0*temp 0*temp 0*temp; 0*temp 0*temp 0*temp];

TwoDOF = 1;

if (TwoDOF)
    
    signpost(verbose,'Two DOF')
    
    l3 = 0;
    m3 = 0;
    
    Ixx3 = 0;
    Iyy3 = 0;
    Izz3 = 0;
    
    T3 = 0;
    
    it1 = 3;
    it2 = 3;
    it3 = 3;
end


%Angles (relative)
alpha = a1;
beta = alpha + a2;
gamma = beta + a3;

p1_0 = [l1*cos(alpha); l1*sin(alpha); 0];
p2_0 = [(l2*cos(beta) + p1_0(1)); (l2*sin(beta) + p1_0(2)); (0 + p1_0(3))];
p3_0 = [ 0; 0; 0];


%% Finding mn_Jvn_JvnT
signpost(verbose,'Finding mn_Jvn_JvnT')

% matrix for Jv1
e11 = diff(p1_0(1), a1);
e12 = diff(p1_0(1), a2);
e13 = diff(p1_0(1), a3);

e21 = diff(p1_0(2), a1);
e22 = diff(p1_0(2), a2);
e23 = diff(p1_0(2), a3);

e31 = diff(p1_0(3), a1);
e32 = diff(p1_0(3), a2);
e33 = diff(p1_0(3), a3);

Jv1 = [e11 e12 e13; e21 e22 e23; e31 e32 e33];

Jv1T = transpose(Jv1);

m1_Jv1_Jv1T = simplify(m1*(Jv1T*Jv1));

% matrix for Jv2
e11 = diff(p2_0(1), a1);
e12 = diff(p2_0(1), a2);
e13 = diff(p2_0(1), a3);

e21 = diff(p2_0(2), a1);
e22 = diff(p2_0(2), a2);
e23 = diff(p2_0(2), a3);

e31 = diff(p2_0(3), a1);
e32 = diff(p2_0(3), a2);
e33 = diff(p2_0(3), a3);

Jv2 = [e11 e12 e13; e21 e22 e23; e31 e32 e33];
Jv2T = transpose(Jv2);

m2_Jv2_Jv2T = simplify(m2*(Jv2T*Jv2));

% matrix for Jv3
e11 = diff(p3_0(1), a1);
e12 = diff(p3_0(1), a2);
e13 = diff(p3_0(1), a3);

e21 = diff(p3_0(2), a1);
e22 = diff(p3_0(2), a2);
e23 = diff(p3_0(2), a3);

e31 = diff(p3_0(3), a1);
e32 = diff(p3_0(3), a2);
e33 = diff(p3_0(3), a3);

Jv3 = [e11 e12 e13; e21 e22 e23; e31 e32 e33];
Jv3T = transpose(Jv3);

m3_Jv3_Jv3T = simplify(m3*(Jv3T*Jv3));

%% Finding Jwn_In_JwnT
signpost(verbose,'Finding Jwn_In_JwnT')

Jw1 = [0 0 0; 0 0 0; 1 0 0];
Jw2 = [0 0 0; 0 0 0; 1 1 0];
Jw3 = [0 0 0; 0 0 0; 1 1 1];

Jw1T = transpose(Jw1);
Jw2T = transpose(Jw2);
Jw3T = transpose(Jw3);

I1 = [Ixx1 0 0; 0 Iyy1 0; 0 0 Izz1];
I2 = [Ixx2 0 0; 0 Iyy2 0; 0 0 Izz2];
I3 = [Ixx3 0 0; 0 Iyy3 0; 0 0 Izz3];

Jw1_I1_Jw1T = Jw1T*I1*Jw1;
Jw2_I2_Jw2T = Jw2T*I2*Jw2;
Jw3_I3_Jw3T = Jw3T*I3*Jw3;

%% Finding Matrix M
signpost(verbose,'Finding Matrix M')

M = Jw1_I1_Jw1T + Jw2_I2_Jw2T + Jw3_I3_Jw3T + m1_Jv1_Jv1T + m2_Jv2_Jv2T + m3_Jv3_Jv3T;


p(M)


%% Finding Matrix G
signpost(verbose,'Finding Matrix G')

g1 = [0; m1*g; 0];
g2 = [0; m2*g; 0];
g3 = [0; m3*g; 0];

Jv1_g1 = (Jv1)*(g1);
Jv2_g2 = (Jv2)*(g2);
Jv3_g3 = (Jv3)*(g3);

G = -1*(Jv1_g1) + -1*(Jv2_g2) + -1*(Jv3_g3);

%% Finding Matrix B and C
signpost(verbose,'Finding Matrix B and C')

gamma = [0*temp 0*temp 0*temp];

a_list = [a1, a2, a3];



for i = 1:it1
    for j = 1:it2
        for k = 1:it3
            
            Mij = M(i,j);
            Mik = M(i,k);
            Mjk = M(j,k);
            
            a_a = a_list(k);
            a_b = a_list(j);
            a_c = a_list(i);
            
            dMij = diff(Mij, a_a);
            dMik = diff(Mik, a_b);
            dMjk = diff(Mjk, a_c);
            
            gamma(k) = dMij + dMik + dMjk;
            
        end
        
        C(i,j) = gamma(1) + gamma(2) + gamma(3);
        
    end
end


%% Finding EOM
signpost(verbose,'Finding EOM')

T = [T1;T2;T3];

a   = [a1;   a2;   a3];
da  = [da1;  da2;  da3];
dda = [dda1; dda2; dda3];



signpost(verbose,'Done: get_EOM_DOF3()')

