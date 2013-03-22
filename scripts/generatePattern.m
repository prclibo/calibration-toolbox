%--------------------------------------------------------------------------
%     Copyright (C) 2013 Bo Li (prclibo@gmail.com)
%     
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
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
