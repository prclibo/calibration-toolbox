path = '../image/5cameras/selected/'; 
fileList = dir([path, '*.png']); 

for i = 1:numel(fileList)
    display(fileList(i).name); 
    index = sscanf(fileList(i).name, '%d-%d.png'); 
    system(['mv ', path, fileList(i).name, ' ', path, num2str(index(2)), '-', num2str(index(1)), '.png']); 
end