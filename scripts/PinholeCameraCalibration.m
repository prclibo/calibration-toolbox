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
classdef PinholeCameraCalibration < CameraCalibrationBase
    methods
        % 
        function obj = PinholeCameraCalibration(width, height, pattern) 
            obj = obj@CameraCalibrationBase(width, height, pattern); 
            obj.camera = PinholeCamera(width, height); 
            obj.maxSmoothError = 10 / 1000 * width; 
        end
    end
    methods(Access = protected)
        % 
        function [gammaGuess, r1All, r2All, tAll] = initializationHelper(obj)

            u0 = obj.camera.width / 2; 
            v0 = obj.camera.height / 2; 
            
            photosValid = find([obj.photosInfo(:).valid]); 
            
            r1All = zeros(3, numel(obj.photosInfo)); 
            r2All = zeros(3, numel(obj.photosInfo)); 
            tAll = zeros(3, numel(obj.photosInfo)); 
            gammaAll = zeros(1, 0); 
            
            % Solve r1, r2 and t(1:2) one by on first
            for i = photosValid
                x = obj.photosInfo(i).patternPoints(:, 1); 
                y = obj.photosInfo(i).patternPoints(:, 2); 
                u = obj.photosInfo(i).photoPoints(:, 1) - u0; 
                v = obj.photosInfo(i).photoPoints(:, 2) - v0; 
                sqrRho = u.^2 + v.^2; 
            
                % Form equation for the Essential Matrix
                M = [-v(:).*x(:), -v(:).*y(:), u(:).*x(:), u(:).*y(:), -v(:), u(:)];
                [~, ~, V] = svd(M);
                

                minReprojectError = inf; 
                
                
                % Note that r11, r21, r12, r22, t1, t2 can be flipped. 
                % So the coeff is used to flipped them. 
                for coeff = [1, -1]
                    r11 = V(1, end) * coeff;
                    r21 = V(3, end) * coeff;
                    r12 = V(2, end) * coeff;
                    r22 = V(4, end) * coeff;
                    t1 = V(5, end) * coeff;
                    t2 = V(6, end) * coeff;
                    
                    r31s = sqrt(roots([1, r11^2 + r21^2 - r12^2 - r22^2, -(r11*r12 + r21*r22)^2]));
                    r31s = r31s(imag(r31s) == 0);
                    r31s = [r31s; -r31s];

                    for r31 = r31s' 
                        r32 = -(r11*r12 + r21*r22) / r31;
                        
                        r1 = [r11; r21; r31];
                        r2 = [r12; r22; r32];
                        
                        scale = 1 ./ norm(r1);
                        r1 = r1 .* scale;
                        r2 = r2 .* scale;
                        t = [t1; t2] .* scale; 
                        
                        
                        
                        % Form equations in Scaramuzza's paper
                        A = [];
                        A(:, 1) = [r1(2).*x + r2(2).*y + t(2);
                            r1(1).*x + r2(1).*y + t(1)];
                        A(:, 2) = [-v; -u];
                        
                        % Operation to avoid bad numerical-condition of A
                        maxA = max(abs(A));
                        A = A ./ repmat(maxA, size(A, 1), 1);
                        
                        B = [v .* (r1(3).*x + r2(3).*y);
                            u .* (r1(3).*x + r2(3).*y)];
                        
                        res = A \ B;
                        res = res ./ maxA';
                        
                        gamma = res(1); 
                        
                        if gamma < 0
                            continue; 
                        end
                        
                        t(3) = res(2);
                        r3 = cross(r1, r2); 
                        
                        R = [r1, r2, r3];                         
                        
                        X = [x'; y']; 
                        X(3, :) = 1; 
                        X = [r1, r2, t] * X; 
                        if sum(X(3, :) < 0) > 0
                            continue; 
                        end
                        
                        obj.camera.fromParamVector([gamma, gamma, 0, u0, v0, 0, 0, 0, 0]); 
                        
                        obj.photosInfo(i).rvec = rodrigues(R); 
                        obj.photosInfo(i).tvec = t; 
                        reprojectError = obj.computeReprojError(i); 
                        reprojectError = mean(sqrt(sum(reshape(reprojectError, 2, []).^2))); 

                        if reprojectError < minReprojectError
                            minReprojectError = reprojectError; 
                            r1All(:, i) = r1;
                            r2All(:, i) = r2;
                            tAll(:, i) = t;
                            gammaAll(i) = gamma;
                        end
                    end
                end
                if isinf(minReprojectError)
                    obj.photosInfo(i).valid = false;
                    display('....Remove 1 invalid photo due to init failure. ');
                end
            end

            gammaGuess = median(gammaAll(gammaAll > 0));            
        end     
    end
end

        
        
        