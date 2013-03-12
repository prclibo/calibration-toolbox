clear all
warning('off','vision:transition:usesOldCoordinates'); 
warning('off', 'MATLAB:nearlySingularMatrix'); 

a1 = imread('../image/09756/sa1.JPG'); 


pattern = imread('../image/pattern.png'); 


p1 = imread('../image/p1.jpg'); 
p2 = imread('../image/p2.jpg'); 
p3 = imread('../image/p3.jpg'); 

% pcc = PinholeCameraCalibration(size(p1, 2), size(p1, 1), pattern); 
% pcc.addPhoto(p1); 
% pcc.addPhoto(p2); 
% pcc.addPhoto(p3); 
% pcc.calibrate(); 
% return; 


% ccc = CataCameraCalibration(size(a1, 2), size(a1, 1), pattern); 
% ccc.addPhoto(b1); 
% ccc.addPhoto(b2); 
% ccc.addPhoto(b3); 
% ccc.addPhoto(b4); 
% ccc.addPhoto(b5); 
% ccc.addPhoto(b6); 
% ccc.addPhoto(b7); 
% ccc.addPhoto(b8); 
% ccc.save('ccc'); 
% % ccc.load('ccc'); 
% ccc.calibrate(); 
% 
% figure, imshow(ccc.camera.undistort(b1, 300, 1200, 1200))
% figure, imshow(ccc.camera.undistort(b8, 300, 1200, 1200))
% return; 


a1 = imread('../image/09756/sa1.JPG'); 

a2 = imread('../image/09756/sa2.JPG'); 
a3 = imread('../image/09756/sa3.JPG'); 
a4 = imread('../image/09756/sa4.JPG'); 

b1 = imread('../image/09749/sa1.JPG'); 
b2 = imread('../image/09749/sa2.JPG'); 
b3 = imread('../image/09749/sa3.JPG'); 
b4 = imread('../image/09749/sa4.JPG'); 

b5 = imread('../image/09749/sa5.JPG'); 
b6 = imread('../image/09749/sa6.JPG'); 
b7 = imread('../image/09749/sa7.JPG'); 
b8 = imread('../image/09749/sa8.JPG'); 


c5 = imread('../image/09756b/sa5.JPG'); 
c6 = imread('../image/09756b/sa6.JPG'); 
c7 = imread('../image/09756b/sa7.JPG'); 
c8 = imread('../image/09756b/sa8.JPG'); 


csc = CameraSystemCalibration('CataCamera', 3, size(a1, 2), size(a1, 1), pattern); 

% csc.addPhoto(1, a1, '1'); 
% csc.addPhoto(1, a2, '2'); 
% csc.addPhoto(1, a3, '3'); 
% csc.addPhoto(1, a4, '4'); 
% 
% csc.addPhoto(2, b1, '1'); 
% csc.addPhoto(2, b2, '2'); 
% csc.addPhoto(2, b3, '3'); 
% csc.addPhoto(2, b4, '4'); 
% csc.addPhoto(2, b5, '5'); 
% csc.addPhoto(2, b6, '6'); 
% csc.addPhoto(2, b7, '7'); 
% csc.addPhoto(2, b8, '8'); 
% 
% csc.addPhoto(3, c5, '5'); 
% csc.addPhoto(3, c6, '6'); 
% csc.addPhoto(3, c7, '7'); 
% csc.addPhoto(3, c8, '8'); 
% 
% csc.save('csc'); 
csc.load('csc'); 
tic
csc.calibrate(); 
toc
% csc.save('csc'); 