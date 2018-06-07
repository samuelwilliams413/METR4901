function [H] = compile_all(DOF, A)
Degrees_Of_Freedom = DOF
Degree_To_Analyse = A

%% REMEMBER EVERYTHING IS IN RADIANS
%%
syms s
Transfer_Functions = collect(get_EOM(DOF),s)
H = get_TF(Transfer_Functions)
sys = H(A)
type = 'PID'
pidTuner(sys,type)
end