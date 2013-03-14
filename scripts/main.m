display('----------------------------------------------------------------------'); 
display('Multiple-Camera Calibration Toolbox'); 

display('----------------------------------------------------------------------'); 
display('### Load Pattern')
path = input('Input the path of the pattern ([] = open file selection dialog):', 's'); 
if isempty(path)
    [file, path] = uigetfile('*.jpg;*.tif;*.png;*.gif;*.bmp'); 
    path = [path, file]; 
end
pattern = imread(path); 
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


display('----------------------------------------------------------------------'); 
display('### Load Images'); 
display('How do you like to load the images?'); 
display('1-Select images all together (Images must be name in form of "camerIndex-timeStamp")'); 
display('2-Select images for each pair of cameras'); 
opt = []; 
while isempty(opt) || (opt ~= 1 && opt ~= 2)
    opt = input('Input the number: '); 
end

photos = struct('image', [], 'camera', [], 'timeStamp', []); 
if opt == 1
    display('### Select images all together'); 
    [files, path] = uigetfile('*.jpg;*.tif;*.png;*.gif;*.bmp', 'MultiSelect', 'On'); 
    for i = 1:numel(files)
        im = imread([path, files(i)]); 
        index = sscanf(fileList(i).name, '%d-%d'); 

        photos(end + 1).image = im; 
        photos(end).camera = index(1); 
        photos(end).timeStamp = index(2); 
    end
else
    
end
display('')