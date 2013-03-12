clear all
warning('off','vision:transition:usesOldCoordinates'); 
warning('off', 'MATLAB:nearlySingularMatrix'); 

pattern = imread('../image/pattern500x680.png'); 
pattern = rgb2gray(pattern); 

% im = imread('../image/mono_left_1362564564997702.bmp'); 
im = imread('../image/mono_front_1362564578438033.bmp'); 
csc = CameraSystemCalibration('cataCamera', 2, size(im, 2), size(im, 1), pattern); 
csc.fixIntrinsics = false; 

csc.addPhoto(1, im, '1'); 