path = '../image/bo-special/'; 
fileList = dir([path, 'right*.bmp']); 

for i = 1:numel(fileList)
    display(fileList(i).name); 
    index = sscanf(fileList(i).name, 'right%d.bmp'); 
    system(['mv ', path, fileList(i).name, ' ', path, num2str(2), '-', num2str(index(1)), '.bmp']); 
end