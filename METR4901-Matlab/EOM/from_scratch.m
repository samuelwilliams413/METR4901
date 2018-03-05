%% function [] = laplaceTransform(torque)
%
%%

function [] = from_scratch()
DOF = 3

p(DOF)

T = get_EOM(DOF);
L = laplaceTransform(T);

end