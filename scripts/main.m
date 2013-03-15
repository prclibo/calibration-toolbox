display('----------------------------------------------------------------------'); 
display('Multiple-Camera Calibration Toolbox'); 
clear all
addpath('utils/'); 


display('----------------------------------------------------------------------'); 
display('### Load Pattern')
display('Input the path of the pattern'); 
[file, path] = uigetfile('*.jpg;*.tif;*.png;*.gif;*.bmp');
path = [path, file];

pattern = imread(path); 
if (size(pattern, 3) > 1)
    pattern = rgb2gray(pattern); 
end

display([path, ' successfully loaded']); 

display('### Resize Pattern')
display('Do you need to resize the pattern?'); 
display('If the pattern resolution is very high, '); 
display('suitable shrinking can help speed up and enhance the feature detection. '); 
scale = input('Input the scale ([] = no resize): '); 
if ~isempty(scale)
    pattern = imresize(pattern, scale); 
    display('Resized'); 
else
    display('Not resized'); 
end

display('----------------------------------------------------------------------'); 
display('### Camera Numbers'); 
nCameras = []; 
while isempty(nCameras)
    nCameras = input('Input the number of cameras in the system: '); 
end
obj = CameraSystemCalibration(nCameras, pattern); 

display('### Camera Type'); 
opt = [];
while isempty(opt) || (opt ~= 1 && opt ~= 2)
    opt = input('Use pinhole (1) or catadioptric (2) model for all cameras (1/2)? '); 
end
if ###


display('----------------------------------------------------------------------'); 
display('### Load Images'); 
photos = repmat(struct('image', [], 'camera', [], 'timeStamp', []), 0, 1);

display('### Select images all together (should be named in form "cameraIndex-timeStamp")');
[files, path] = uigetfile('*.jpg;*.tif;*.png;*.gif;*.bmp', 'MultiSelect', 'On');
for i = 1:numel(files)
    im = imread([path, files{i}]);
    index = sscanf(files{i}, '%d-%d');
    
    photos(end + 1).image = im;
    photos(end).camera = index(1);
    photos(end).timeStamp = index(2);
end
display([num2str(numel(photos)), 'images loaded']); 

display('----------------------------------------------------------------------'); 
display('### Process Images'); 
for i = 1:numel(photos)
    obj.addPhoto(photos(i).camera, photos(i).image, num2str(photos(i).timeStamp)); 
end

display('----------------------------------------------------------------------'); 
display('### Process Calibration'); 
obj.calibrate(); 
