
function [LB,UB] = read_bounds(filename)

boundstable = readtable(filename);
bounds = table2array(boundstable(:,2:3));
    
LB = bounds(:,1);
UB = bounds(:,2);

end