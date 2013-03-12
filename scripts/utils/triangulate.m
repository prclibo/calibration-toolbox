function X = triangulate(P1, P2, x1, x2) 

x1 = x1(1:2, :); 
x2 = x2(1:2, :); 
x1(3, :) = 1; 
x2(3, :) = 1; 

n = size(x1, 2); 
X = zeros(3, n); 
for i = 1:n
    A = [skew(x1(:, i)) * P1; 
         skew(x2(:, i)) * P2]; 
    [~, ~, V] = svd(A); 
    X(:, i) = V(1:3, end) ./ V(end, end); 
end

end