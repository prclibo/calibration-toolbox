%--------------------------------------------------------------------------
%     Copyright (C) 2013 Bo Li (prclibo@gmail.com)
%     
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
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
        loadIntrinsics(obj); 
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