path = '../image/temp/cam3/'; 
fileList = dir([path, '*.bmp']); 

for i = 1:numel(fileList)
    display(fileList(i).name); 
    index = sscanf(fileList(i).name, '%dimg%d.bmp'); 
    system(['mv ', path, fileList(i).name, ' ', path, num2str(index(1)), 'img', num2str(index(2) + 75), '.bmp']); 
end