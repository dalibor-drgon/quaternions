
function [A, b] = create_matrices(data)
    A = [];
    b = [];
    for i = 2:size(data, 1)
        A = [A; 2 * (data(i, :) - data(1, :))];
        b = [b; dot(data(i, :), data(i, :)) - dot(data(1, :), data(1, :))];
    end
end

function ret = solve(data) 
    [A, b] = create_matrices(data)
    ret = inv(transpose(A) * A) * transpose(A) * b;
end

data=[-4.684658 -8.655601 0.680186; 0.402363 -7.271279 -6.389912; -7.285649 0.900527 -6.547983; 0.996328 -0.986748 9.829160; ];

solve(data)
