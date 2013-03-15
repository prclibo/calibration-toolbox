clear all
addpath('utils/')

path = '../image/box-03-13/'
fileList = dir([path, '*.bmp']); 

pattern = imread('../image/patternO.png'); 
% pattern = pattern(600:end, 800:end); 
% pattern = imfilter(pattern, fspecial('gaussian', [7 7])); 

csc = CameraSystemCalibration(4, pattern); 
csc.fixIntrinsics = false; 
csc.setCameraType(1, 'pinholecamera', 752, 480); 
csc.setCameraType(2:4, 'pinholecamera', 640, 480); 

csc.cameraCalibrations(1).minMatchedPoints = 9; 
csc.cameraCalibrations(2).minMatchedPoints = 9; 
csc.cameraCalibrations(3).minMatchedPoints = 9; 
csc.cameraCalibrations(4).minMatchedPoints = 9; 

for i = 1:numel(fileList)
    display(fileList(i).name); 
    index = sscanf(fileList(i).name, '%dimg%d.bmp'); 
    im = imread([path, fileList(i).name]); 
%     im = imresize(im, 0.5); 
    csc.addPhoto(index(1) + 1, im, num2str(index(2))); 
end

csc.save('csc'); 
% csc.load('csc'); 
tic 
csc.calibrate(); 
toc