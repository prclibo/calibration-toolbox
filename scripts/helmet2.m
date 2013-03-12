clear all

path = '../image/5cameras/selected/'; 
fileList = dir(path); 

pattern = imread('../image/pattern.png'); 

csc = CameraSystemCalibration('catacamera', 2, 856, 480, pattern); 
csc.fixIntrinsics = false; 
% for i = 4:numel(fileList)
%     display(fileList(i).name); 
%     index = sscanf(fileList(i).name, '%d-%d.png'); 
%     
%     if index(2) ~= 1 && index(2) ~= 2
%         continue; 
%     end
%     
%     im = imread([path, fileList(i).name]); 
%     csc.addPhoto(index(2), im, num2str(index(1))); 
% end
% 
% csc.save('csc'); 
csc.load('csc'); 
tic 
csc.calibrate(); 
toc

csc.cameraCalibrations(1).showPoints(); 
csc.cameraCalibrations(2).showPoints(); 