clear all
addpath('utils/')
warning('off','vision:transition:usesOldCoordinates'); 
warning('off', 'MATLAB:nearlySingularMatrix'); 

pattern = imread('../image/pattern500x680.png'); 
pattern = rgb2gray(pattern); 

l0 = imread('../image/bo-special/1-0.bmp'); 
l1 = imread('../image/bo-special/1-1.bmp'); 
l2 = imread('../image/bo-special/1-2.bmp'); 
l3 = imread('../image/bo-special/1-3.bmp'); 
l4 = imread('../image/bo-special/1-4.bmp'); 
l5 = imread('../image/bo-special/1-5.bmp'); 
l6 = imread('../image/bo-special/1-6.bmp'); 
l7 = imread('../image/bo-special/1-7.bmp'); 
l8 = imread('../image/bo-special/1-8.bmp'); 
l9 = imread('../image/bo-special/1-9.bmp'); 
l10 = imread('../image/bo-special/1-10.bmp'); 
l11 = imread('../image/bo-special/1-11.bmp'); 
l12 = imread('../image/bo-special/1-12.bmp'); 
l13 = imread('../image/bo-special/1-13.bmp'); 
l14 = imread('../image/bo-special/1-14.bmp'); 
l15 = imread('../image/bo-special/1-15.bmp'); 
l16 = imread('../image/bo-special/1-16.bmp'); 
l17 = imread('../image/bo-special/1-17.bmp'); 
l18 = imread('../image/bo-special/1-18.bmp'); 
l19 = imread('../image/bo-special/1-19.bmp'); 
l20 = imread('../image/bo-special/1-20.bmp'); 
l21 = imread('../image/bo-special/1-21.bmp'); 
l22 = imread('../image/bo-special/1-22.bmp'); 
l23 = imread('../image/bo-special/1-23.bmp'); 
l24 = imread('../image/bo-special/1-24.bmp'); 
l25 = imread('../image/bo-special/1-25.bmp'); 
l26 = imread('../image/bo-special/1-26.bmp'); 
l27 = imread('../image/bo-special/1-27.bmp'); 
l28 = imread('../image/bo-special/1-28.bmp'); 
l29 = imread('../image/bo-special/1-29.bmp'); 

pcc = PinholeCameraCalibration(size(l1, 2), size(l1, 1), pattern); 

pcc.addPhoto(l0); 
pcc.addPhoto(l1); 
pcc.addPhoto(l2); 
pcc.addPhoto(l3); 
pcc.addPhoto(l4); 
pcc.addPhoto(l5); 
pcc.addPhoto(l6); 
pcc.addPhoto(l7); 
pcc.addPhoto(l8); 
pcc.addPhoto(l9); 
pcc.addPhoto(l10); 
pcc.addPhoto(l11); 
pcc.addPhoto(l12); 
pcc.addPhoto(l13); 
pcc.addPhoto(l14); 
pcc.addPhoto(l15); 
pcc.addPhoto(l16); 
pcc.addPhoto(l17); 
pcc.addPhoto(l18); 
pcc.addPhoto(l19); 

pcc.calibrate(); 
pcc.plotPatternBound(4); 