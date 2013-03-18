function pattern = generatePattern(N, M)

% M = 2500; 
% N = 3400; 

pattern = 0; 
count = 0; 

m = 5; 
% randseed(3); 
while m < M
    n = round(N / M * m); 
    
    rnd = rand(n, m); 
    rnd = imresize(rnd, [N, M]); 
    rnd = (rnd - min(rnd(:))) / (max(rnd(:)) - min(rnd(:))); 
    
    pattern = pattern + rnd; 
    count = count + 1; 
    m = m * 2; 
end

pattern = pattern ./count; 
pattern = histeq(pattern); 

end
