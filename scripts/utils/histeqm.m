function J = histeqm(I, mask, H)

if nargin < 3 || length(H) ~= 256
    H = ones(1, 256); 
end

H = H ./ sum(H); 

if nargin < 2 
    mask = true(size(I)); 
end


if (size(I, 3) > 1)
    I = rgb2gray(I); 
end

Id = double(I); 
Id = (Id - min(Id(:))) ./ (max(Id(:)) - min(Id(:)) + 1e-5) * 256; 
Id = floor(Id); 
hI = histc(Id(mask), 0:256) ./ sum(mask(:)); 

cI = cumsum(hI); 
C = cumsum(H); 

color = zeros(256, 1); 
for i = 1:256
    color(i) = sum(C < cI(i)); 
end

J = color(Id(:) + 1) ./ 255; 
J = reshape(J, size(Id)); 
J = J .* mask + Id .* ~mask; 


if strcmpi(class(I), 'uint8')
    J = im2uint8(J); 
end

end