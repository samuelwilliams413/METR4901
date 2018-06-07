%% function [ ] = signpost(flag, statement)
%
%%

function [ ] = signpost(flag, statement)
%% Signpost
% Takes an input flag, if true the given statement is printed (used for
% debugging and checking progress). This allows for print dialogues to be
% controlled via centralised variables

if(flag)
    disp(statement)
end

end

