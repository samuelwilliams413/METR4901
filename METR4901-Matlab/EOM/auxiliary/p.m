%% function [ ] = p(variable)
%
%%

function [ ] = p(variable)
%% p - Quick Print
% This function takes a variable, retrieves the variable name, and then 
% obtains the line in the parent function from which it was called. Next 
% the variable name, line number, and variable are printed to the console

% Get variable name
variable_name = inputname(1);

% Get current line (in parent -> 2) from stack
stack = dbstack();
current_line = stack(2).line;

% Print
disp(['"',variable_name,'" at line ', num2str(current_line),': '])
disp(variable)

end

