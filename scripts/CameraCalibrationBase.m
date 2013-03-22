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
classdef CameraCalibrationBase < handle & matlab.mixin.Heterogeneous
    properties
        
        % Settings
        minMatchedPoints; 
        maxInitReprojectError; 
        maxInlierError
        maxSmoothError; 
        maxFinalReprojectError; 
        
        intrinsicsFile; 
        
        % Parameters
        camera; 
        
        % Data
        pattern; 
        patternPoints; 
        patternFeatures; 
        photosInfo; 
    end
    
    methods(Access = public)
        % 
        function obj = CameraCalibrationBase(width, height, pattern) 
            obj.pattern = pattern; 
            obj.photosInfo = []; 
            obj.camera = []; 
            
            obj.minMatchedPoints = max(15, 20 / 1000 * width); 
            obj.maxInitReprojectError = 20 / 1200 * width; 
            obj.maxFinalReprojectError = 1.5; 
            obj.maxInlierError = 1; 
            obj.maxSmoothError = 80 / 1000 * width; 
            
            obj.intrinsicsFile = []; 

            if ~isempty(pattern)
                patternKeyPoints = detectSURFFeatures(obj.pattern, 'NumOctaves', 3, 'NumScaleLevels', 3); 
                [obj.patternFeatures, obj.patternPoints] = extractFeatures(obj.pattern, patternKeyPoints); 
            else 
                obj.patternFeatures = []; 
                obj.patternPoints = []; 
            end

        end
        
        %
        function obj = save(obj, name)
            minMatchedPoints = obj.minMatchedPoints;
            maxInitReprojectError = obj.maxInitReprojectError;
            camera = obj.camera;
            pattern = obj.pattern;
            photosInfo = obj.photosInfo;
            patternFeatures = obj.patternFeatures; 
            patternPoints = obj.patternPoints; 
            
            save(name, ...
                 'minMatchedPoints', ...
                 'maxInitReprojectError', ...
                 'camera', ...
                 'pattern', ...
                 'photosInfo', ...
                 'patternFeatures', ...
                 'patternPoints'); 
        end
        
        %
        function obj = load(obj, name)
            s = load(name); 
            obj.minMatchedPoints = s.minMatchedPoints;
            obj.maxInitReprojectError = s.maxInitReprojectError;
            obj.camera = s.camera;
            obj.pattern = s.pattern;
            obj.photosInfo = s.photosInfo;
            obj.patternFeatures = s.patternPoints; 
            obj.patternPoints = s.patternPoints; 
        end
        
        %
        function obj = setIntrinsics(obj, fileName)
            obj.intrinsicsFile = fileName; 
        end
        
        % 
        function [photoIndex, valid] = addPhoto(obj, photo)
            if (size(photo, 3) > 1) 
                photo = rgb2gray(photo); 
            end
            
            photo1 = photo; 
            photo2 = histeq(photo);
          
            obj.photosInfo(end + 1).photo = photo; 
            photoIndex = numel(obj.photosInfo); 
                        
            photoKeyPoints1 = detectSURFFeatures(photo1, 'NumOctaves', 8, 'MetricThreshold', 200); 
            photoKeyPoints2 = detectSURFFeatures(photo2, 'NumOctaves', 8, 'MetricThreshold', 200); 

                        
            patternFeatures = obj.patternFeatures; 
            patternPoints = obj.patternPoints; 
            [photoFeatures1, photoPoints1] = extractFeatures(photo1, photoKeyPoints1); 
            [photoFeatures2, photoPoints2] = extractFeatures(photo2, photoKeyPoints2); 
            
            if numel(photoPoints1) > numel(photoPoints2)
                photoPoints = photoPoints1; 
                photoFeatures = photoFeatures1; 
            else
                photoPoints = photoPoints2; 
                photoFeatures = photoFeatures2; 
            end

            % Descriptor matching
            indexPairs = matchFeatures(patternFeatures, photoFeatures, 'Method', 'NearestNeighborRatio'); 
            display(['....Matches: ', num2str(size(indexPairs, 1))]); 
            
            if (size(indexPairs, 1) < max(obj.minMatchedPoints, 8))
                obj.photosInfo(end).valid = false; 
                valid = false; 
                display('....Invalid photo due to too few matches with pattern. '); 
                return; 
            end
            
            patternPoints = patternPoints(indexPairs(:, 1));
            photoPoints = photoPoints(indexPairs(:, 2));
            
            patternPoints = patternPoints(:).Location;
            photoPoints = photoPoints(:).Location;

            % Fundamental matrix check
            [~, inliersMaskF] = estimateFundamentalMatrix(patternPoints, photoPoints, 'Method', 'RANSAC', ...
                                                'DistanceThreshold', obj.maxInlierError);
            
            estimator = vision.GeometricTransformEstimator;
            estimator.Transform = 'projective';
            estimator.AlgebraicDistanceThreshold = obj.maxInlierError;
            [~, inliersMaskH] = estimator.step(patternPoints, photoPoints);  
            
            if (sum(inliersMaskH) > sum(inliersMaskF) * 0.8)
                patternPoints = patternPoints(inliersMaskH, :);
                photoPoints = photoPoints(inliersMaskH, :);
                display(['....Matches after Homog. Check: ', num2str(sum(inliersMaskF))]);
            else
                patternPoints = patternPoints(inliersMaskF, :);
                photoPoints = photoPoints(inliersMaskF, :);
                display(['....Matches after Fundam. Check: ', num2str(sum(inliersMaskF))]);
            end
                        
            if (size(patternPoints, 1) < max(obj.minMatchedPoints, 4))
                obj.photosInfo(end).valid = false; 
                valid = false; 
                display(['....Invalid photo due to too few inliers by Fundam./Homog. Matrix check: ', num2str(size(patternPoints, 1))]); 
                return; 
            end
            
            
            % Further homography check
            estimator = vision.GeometricTransformEstimator;
            estimator.Transform = 'projective';
            estimator.AlgebraicDistanceThreshold = obj.maxSmoothError;
            [~, inliersMask] = estimator.step(patternPoints, photoPoints); 
            
            display(['....Matches after smoothness Check: ', num2str(sum(inliersMask))]);
            if (sum(inliersMask) < obj.minMatchedPoints)
                obj.photosInfo(end).valid = false; 
                valid = false; 
                display('....Invalid photo due to too few inliers by coarse Homography check. '); 
                return; 
            end
            
            
            patternPoints = patternPoints(inliersMask, :);
            photoPoints = photoPoints(inliersMask, :);
            
            obj.photosInfo(end).valid = true; 
            obj.photosInfo(end).patternPoints = patternPoints; 
            obj.photosInfo(end).photoPoints = photoPoints; 
            obj.photosInfo(end).nPoints = size(patternPoints, 1); 
            
            valid = true; 
            
            display(['....', num2str(size(patternPoints, 1)), ' features kept']); 
%             figure, showMatchedFeatures(obj.pattern, photo, patternPoints, photoPoints, 'montage');             
            
            
        end
        
        % 
        function obj = calibrate(obj)
            obj.initializeCalibration();
            if isempty(obj.intrinsicsFile)
                obj.optimizeCalibration(); 
            end
        end

        %
        function obj = setPoseVector(obj, p, photoIndex)
            if isempty(photoIndex)
                photosValid = find([obj.photosInfo(:).valid]);
                for k = 1:numel(photosValid)
                    i = photosValid(k);
                    rvec = p((k - 1) * 6 + (1:3));
                    tvec = p((k - 1) * 6 + (4:6));
                    obj.photosInfo(i).rvec = rvec;
                    obj.photosInfo(i).tvec = tvec(:);
                end
            else
                rvec = p(1:3);
                tvec = p(4:6);
                obj.photosInfo(photoIndex).rvec = rvec;
                obj.photosInfo(photoIndex).tvec = tvec(:);
            end
        end
        
        % 
        function p = getPoseVector(obj, photoIndex)
            if isempty(photoIndex)
                photosValid = find([obj.photosInfo(:).valid]); 
                p = []; 
                for k = 1:numel(photosValid)
                    i = photosValid(k); 
                    rvec = obj.photosInfo(i).rvec; 
                    tvec = obj.photosInfo(i).tvec; 
                    p = [p; rvec(:); tvec(:)]; 
                end
            else
                rvec = obj.photosInfo(photoIndex).rvec;
                tvec = obj.photosInfo(photoIndex).tvec;
                p = [rvec(:); tvec(:)]; 
            end
        end
        
        % 
        function [error, jacobPose, jacobCamera] = computeReprojError(obj, photoIndex)
            rvec = obj.photosInfo(photoIndex).rvec; 
            tvec = obj.photosInfo(photoIndex).tvec; 
            
            patternPoints = double(obj.photosInfo(photoIndex).patternPoints); 
            patternPoints = patternPoints';
            patternPoints(3, :) = 0;
            
            photoPoints = double(obj.photosInfo(photoIndex).photoPoints);
            
            if nargout < 2
                projectedPoints = obj.camera.projectPoints(patternPoints, rvec, tvec); 
            elseif nargout < 3
                [projectedPoints, J_pose] = obj.camera.projectPoints(patternPoints, rvec, tvec);
                jacobPose = J_pose; 
            else
                [projectedPoints, J_pose, J_camera] = obj.camera.projectPoints(patternPoints, rvec, tvec);
                jacobPose = J_pose;
                jacobCamera = J_camera;
            end
            
%             projectedPoints = projectedPoints';
%             error = sum((photoPoints - projectedPoints) .^ 2, 2); 
            error = projectedPoints - photoPoints'; 
            error = error(:); 
        end
        
        %
        function showPoints(obj)
            photosValid = find([obj.photosInfo(:).valid]); 
            
            photoPoints = []; 
            projectedPoints = []; 
            
            for k = 1:numel(photosValid)
                i = photosValid(k); 
                photoPoints = [photoPoints; obj.photosInfo(i).photoPoints]; 
                rvec = obj.photosInfo(i).rvec;
                tvec = obj.photosInfo(i).tvec;
                patternPoints = double(obj.photosInfo(i).patternPoints); 
                patternPoints = patternPoints';
                patternPoints(3, :) = 0;
                projectedPoints = [projectedPoints; obj.camera.projectPoints(patternPoints, rvec, tvec)'];
            end
            
            figure, 
            plot(photoPoints(:, 1), photoPoints(:, 2), '.b'); 
            
            hold on
            plot(projectedPoints(:, 1), projectedPoints(:, 2), '.g'); 
            
        end
        
        %
        function plotPatternBound(obj, photoIndex)
            width = size(obj.pattern, 2); 
            height = size(obj.pattern, 1); 
            
            top(1, :) = 1:width; 
            top(2, :) = 1; 
            right(2, :) = 1:height; 
            right(1, :) = width; 
            bottom(1, :) = width:-1:1; 
            bottom(2, :) = height; 
            left(2, :) = height:-1:1;
            left(1, :) = 1; 
            
            points = [top, right, bottom, left]; 
            points(3, :) = 1; 
            
            photo = obj.photosInfo(photoIndex).photo; 
            rvec = obj.photosInfo(photoIndex).rvec; 
            tvec = obj.photosInfo(photoIndex).tvec; 
            project = obj.camera.projectPoints(points, rvec, tvec); 
            figure, imshow(photo), hold on
            plot(project(1, :), project(2, :), 'linewidth', 2); 
        end

        %
        function render = reprojectPattern(obj, photoIndex)
            raw = obj.photosInfo(photoIndex).photo; 
            width = size(obj.pattern, 2); 
            height = size(obj.pattern, 1); 
            rvec = obj.photosInfo(photoIndex).rvec; 
            tvec = obj.photosInfo(photoIndex).tvec;             
            
            tform = maketform('custom', 2, 2, [], @remap, []); 
            render = imtransform(raw, tform, 'xdata', [1, width], 'ydata', [1, height]); 
            
            function out = remap(in, tdata)
                in(:, 3) = 0; 
                in = in'; 
                out = obj.camera.projectPoints(in, rvec, tvec); 
                out = out'; 
            end
        end
    end
    
    methods (Access = protected)
        
        %
        function [gammaGuess, r1All, r2All, tAll] = initializationHelper(obj)
        end
                
        % 
        function obj = initializeCalibration(obj)
            u0 = obj.camera.width / 2;
            v0 = obj.camera.height / 2; 
            
            [gammaGuess, r1All, r2All, tAll] = initializationHelper(obj); 

            if isempty(obj.intrinsicsFile)
                obj.camera.fromParamVector([gammaGuess, gammaGuess, 0, u0, v0, 1, 0, 0, 0, 0]); 
            else
                obj.camera.loadIntrinsics(obj.intrinsicsFile); 
            end
            
            photosValid = find([obj.photosInfo(:).valid]); 
            for k = 1:numel(photosValid)
                i = photosValid(k); 

                r1 = r1All(:, i); 
                r2 = r2All(:, i); 
                t = tAll(:, i); 
                R = [r1, r2, cross(r1, r2)]; 
                
                obj.photosInfo(i).rvec = rodrigues(R);
                obj.photosInfo(i).tvec = t;
                
                reprojectError = obj.pnpOptimization(i);

                display(['....Init error for ', num2str(i), ' = ', num2str(reprojectError)]); 
                
                if reprojectError > obj.maxInitReprojectError
                    obj.photosInfo(i).valid = false; 
                    display('........Remove 1 invalid photo due to too high init reprojection error. '); 
                    continue;                     
                end

            end            

        end
        
        %
        function obj = optimizeCalibration(obj)

            
            photosValid = find([obj.photosInfo(:).valid]); 
             
            p = [obj.camera.toParamVector(); 
                 obj.getPoseVector([])]; 
            
            objective = @(x) obj.calibrationObjective(x);             
            
            p = double(p); 
%             options=optimset('Display','iter',...
%                  'DerivativeCheck','on',...
%                  'Jacobian','on',...
%                  'MaxIter',10000, 'MaxFunEvals', 20000, 'TolX', 1e-6, 'TolFun', 1e-6, 'Algorithm', 'trust-region-reflective'); 
            options=optimset('Display','off',...
                 'DerivativeCheck','off',...
                 'Jacobian','on',...
                 'MaxIter',10000, 'MaxFunEvals', 20000, 'TolX', 1e-6, 'TolFun', 1e-6, 'Algorithm', 'levenberg-marquardt'); 
 
            [p_res, ~, error] = lsqnonlin(objective, p, [], [], options); 
            error = mean(sqrt(sum(reshape(error, 2, []).^2))); 
            obj.camera.fromParamVector(p_res); 
            
            offset = obj.camera.nParams; 
            obj.setPoseVector(p_res(offset + 1:end), []); 
            
            %%%
            for k = 1:numel(photosValid)
                i = photosValid(k); 
                each_error = obj.computeReprojError(i); 
                each_error = mean(sqrt(sum(reshape(each_error, 2, []).^2))); 
                display(['....Final error for photo', num2str(i), ' = ', num2str((each_error))]); 
                if (each_error > obj.maxFinalReprojectError)
                    obj.photosInfo(i).valid = false;
                    display('........Removed 1 photo due to too high final reproject error. ');
                end
            end
            %%%
            
            display('....Optimization result: '); 
            display(['........error = ', num2str(error)]); 
            
            
        end
        

        
        %
        function [error, jacobCameraPose] = calibrationObjective(obj, p)
            
            photosValid = find([obj.photosInfo(:).valid]); 
                       
            obj.camera.fromParamVector(p); 
            offset = obj.camera.nParams; 
            
            pp = p(offset + 1:end); 
            obj.setPoseVector(pp, []); 
            
            error = []; 
            jacobCamera = []; 
            jacobPose = []; 
            for k = 1:numel(photosValid)
                i = photosValid(k); 
                nPoints = obj.photosInfo(i).nPoints; 
                
                [each_error, J_pose, J_camera] = obj.computeReprojError(i); 
                error = [error; each_error]; 
                jacobCamera = [jacobCamera; J_camera]; 
                jacobPose(end + (1:nPoints * 2), end + (1:6)) = J_pose; 
            end
            
            jacobCameraPose = [jacobCamera, jacobPose]; 
            jacobCameraPose = sparse(jacobCameraPose); 
            
        end
        
        % 
        function error = pnpOptimization(obj, photoIndex)
            p = obj.getPoseVector(photoIndex); 
            p = double(p); 
            
            objective = @(x) obj.pnpObjective(photoIndex, x); 
%             options=optimset('Display','off',...
%                  'DerivativeCheck','on',...
%                  'Jacobian','on',...
%                  'MaxIter',10000, 'Algorithm', 'trust-region-reflective'); 
            options=optimset('Display','off',...
                 'DerivativeCheck','off',...
                 'Jacobian','on',...
                 'MaxIter',10000, 'MaxFunEvals', 20000, 'TolX', 1e-6, 'TolFun', 1e-6, 'Algorithm', 'levenberg-marquardt'); 

            bound = [pi; pi; pi; inf; inf; inf]; 
            [p_res, ~, error] = lsqnonlin(objective, p, [], [], options); 
            obj.setPoseVector(p_res, photoIndex); 
            
            error = mean(sqrt(sum(reshape(error, 2, []).^2))); 
            
        end
        
        % 
        function [error, jacobPose] = pnpObjective(obj, photoIndex, p)
            obj.setPoseVector(p, photoIndex); 

            [error, jacobPose] = obj.computeReprojError(photoIndex); 
        end
        
        
    end
end

        
        
        