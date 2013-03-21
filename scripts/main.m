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
    display(['Resized to ', num2str(scale * 100), '%']); 
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
CAMERA_TYPE={'PinholeCamera', 'CataCamera'}; 
cameraType = cell(nCameras, 1); 

opt = [];
while isempty(opt) || (opt ~= 1 && opt ~= 2)
    opt = input('Use pinhole (1) or catadioptric (2) model for Camera #1 (1/2)? '); 
end
cameraType{1} = CAMERA_TYPE{opt}; 

if (nCameras > 1)
    while ~strcmpi(opt, 'Y') && ~strcmpi(opt, 'N') && ~isempty(opt)
        opt = input('Use the same model for the rest of the cameras (Y/N, []=Y)? ', 's'); 
    end
    if strcmpi(opt, 'N')
        for i = 2:nCameras
            opt = [];
            while isempty(opt) || (opt ~= 1 && opt ~= 2)
                opt = input(['Use pinhole (1) or catadioptric (2) model for Camera #', num2str(i), ' (1/2)? ']);  
            end
            cameraType{i} = CAMERA_TYPE{opt}; 
        end
    else
        for i = 2:nCameras
            cameraType{i} = cameraType{1};
        end
    end
end


display('----------------------------------------------------------------------'); 
display('### Load Images'); 
photos = repmat(struct('image', [], 'camera', [], 'timeStamp', []), 0, 1);
width = zeros(nCameras, 1); 
height = zeros(nCameras, 1); 

display('### Select images all together (should be named in form "cameraIndex-timeStamp")');
[files, path] = uigetfile('*.jpg;*.tif;*.png;*.gif;*.bmp', 'MultiSelect', 'On');
if ~iscell(files)
    error('You cannot only select one file! '); 
end
for i = 1:numel(files)
    im = imread([path, files{i}]);
    index = sscanf(files{i}, '%d-%d');
    
    photos(end + 1).image = im;
    photos(end).camera = index(1);
    photos(end).timeStamp = num2str(index(2));
    
    width(index(1)) = size(photos(end).image, 2); 
    height(index(1)) = size(photos(end).image, 1); 
end
display([num2str(numel(photos)), ' images loaded']); 

izeros = find(width == 0); 
if ~isempty(izeros)
    error(['No image loaded for Camera #', num2str(izeros(1)), '! ']); 
end

display('----------------------------------------------------------------------'); 
display('### Process Images'); 
% Initialize mono camera calibration models first
for i = 1:nCameras
    obj.setCameraType(i, cameraType{i}, width(i), height(i)); 
end

for i = 1:numel(photos)
    obj.addPhoto(photos(i).camera, photos(i).image, num2str(photos(i).timeStamp)); 
end
clear photos; 

display('----------------------------------------------------------------------'); 
display('### Process Calibration'); 
obj.calibrate(); 
display('### Calibration finished'); 


display('----------------------------------------------------------------------'); 
display('### Intrinsics: '); 
obj.outputIntrinsics(); 
display('### Extrinsics: '); 
obj.outputExtrinsics(); 

input('Press ENTER to visualize camera poses plot and pose graph'); 
obj.visualizeObjects(); 
obj.visualizeGraph(); 


