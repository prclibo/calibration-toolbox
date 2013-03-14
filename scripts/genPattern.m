M = 2500; 
N = 3400; 

pattern = 0; 
count = 0; 

m = 10; 
randseed(3); 
while m < M
    n = round(N / M * m); 
    
    rnd = rand(n, m); 
%     rnd = imfilter(rnd, ones(3)/9); 
    rnd = imresize(rnd, [N, M]); 
%     rnd = histeq(rnd);     
    rnd = (rnd - min(rnd(:))) / (max(rnd(:)) - min(rnd(:))); 
    
    pattern = pattern + rnd; 
    count = count + 1; 
    m = m * 2; 
end

pattern = pattern ./count; 
pattern = histeq(pattern); 
% pattern = imfilter(pattern, fspecial('gaussian')); 
% pattern = (pattern - min(pattern(:))) / (max(pattern(:)) - min(pattern(:))); 
imwrite(pattern, 'pattern.png')
imshow(pattern, []); 