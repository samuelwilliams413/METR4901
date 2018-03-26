function [] = compile_all(DOF)
syms s
Transfer_Functions = collect(get_EOM(DOF),s)
get_TF(Transfer_Functions)
end