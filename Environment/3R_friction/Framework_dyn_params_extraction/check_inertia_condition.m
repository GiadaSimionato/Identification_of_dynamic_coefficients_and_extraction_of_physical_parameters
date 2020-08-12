% [Code by Claudio Gaz.]

function loss = check_inertia_condition(I,loss,penalty)

cond = trace(I)/2 - max(eig(I));
if cond < 0
    loss = loss - penalty*cond;
end    