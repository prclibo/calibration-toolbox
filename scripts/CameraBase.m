classdef CameraBase < handle & matlab.mixin.Heterogeneous
    properties
        width
        height
        
        nParams = 2; 
    end
    
    methods
        %
        function obj = CameraBase(width, height)
            obj.width = width; 
            obj.height = height;             
        end
    end
    
    methods (Abstract)
        toParamVector(obj); 
        fromParamVector(obj); 
        projectPoints(obj); 
        outputIntrinsics(obj); 
    end
    
    methods
        function undist = undistort(cataCamera, raw, focal, width, height)

            tform = maketform('custom', 2, 2, [], @remap, []); 
            undist = imtransform(raw, tform, 'xdata', [1, width], 'ydata', [1, height]); 
            
            function out = remap(in, tdata)
                in(:, 1) = in(:, 1) - width / 2; 
                in(:, 2) = in(:, 2) - height / 2; 
                in(:, 3) = focal; 
                in = in'; 
                out = cataCamera.projectPoints(in, [0; 0; 0], [0; 0; 0]); 
                out = out'; 
            end
        end
    end
end 