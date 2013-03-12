classdef CameraCalibrationBase < handle & matlab.mixin.Heterogeneous
    properties
        
        % Settings
        minMatchedPoints; 
        maxInitReprojectError; 
        maxFundaMatError
        maxHomographyError; 
        maxFinalReprojectError; 
        
        % Parameters
        camera; 
        
        % Data
        pattern; 
        photosInfo; 
    end
    
    methods (Abstract)
        initializeCalibration(obj)
    end
    
    methods
        % 
        function obj = CameraCalibrationBase(width, height, pattern)
            addpath('utils');
            
            obj.pattern = pattern; 
            obj.photosInfo = []; 
            obj.camera = []; 
            
            obj.minMatchedPoints = 15 / 1000 * width; 
            obj.maxInitReprojectError = 20 / 1200 * width; 
            obj.maxFinalReprojectError = 1.5; 
            obj.maxFundaMatError = 1; 
            obj.maxHomographyError = 80 / 1000 * width; 
            
        end
        
        %
        function obj = save(obj, name)
            minMatchedPoints = obj.minMatchedPoints;
            maxInitReprojectError = obj.maxInitReprojectError;
            camera = obj.camera;
            pattern = obj.pattern;
            photosInfo = obj.photosInfo;
            save(name, ...
                 'minMatchedPoints', ...
                 'maxInitReprojectError', ...
                 'camera', ...
                 'pattern', ...
                 'photosInfo'); 
        end
        
        %
        function obj = load(obj, name)
            s = load(name); 
            obj.minMatchedPoints = s.minMatchedPoints;
            obj.maxInitReprojectError = s.maxInitReprojectError;
            obj.camera = s.camera;
            obj.pattern = s.pattern;
            obj.photosInfo = s.photosInfo;
        end
        
        % 
        function [photoIndex, valid] = addPhoto(obj, photo)
            if (size(photo, 3) > 1) 
                photo = rgb2gray(photo); 
            end
            
            photo = histeq(photo); 
          
            obj.photosInfo(end + 1).photo = photo; 
            photoIndex = numel(obj.photosInfo); 
                        
            patternKeyPoints = detectSURFFeatures(obj.pattern, 'NumOctaves', 16); 
            photoKeyPoints = detectSURFFeatures(photo, 'NumOctaves', 8); 

            [patternFeatures, patternPoints] = extractFeatures(obj.pattern, patternKeyPoints);
            [photoFeatures, photoPoints] = extractFeatures(photo, photoKeyPoints);
            

            % Descriptor matching
            indexPairs = matchFeatures(patternFeatures, photoFeatures, 'Method', 'NearestNeighborRatio');
            
            if (size(indexPairs, 1) < obj.minMatchedPoints)
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
            [~, inliersMask] = estimateFundamentalMatrix(patternPoints, photoPoints, 'Method', 'RANSAC', ...
                                                'DistanceThreshold', obj.maxFundaMatError);
            
            if (sum(inliersMask) < obj.minMatchedPoints)
                obj.photosInfo(end).valid = false; 
                valid = false; 
                display('....Invalid photo due to too few inliers by Fundamental Matrix check. '); 
                return; 
            end
            
            patternPoints = patternPoints(inliersMask, :);
            photoPoints = photoPoints(inliersMask, :);
            
            % Further homography check
            estimator = vision.GeometricTransformEstimator;
            estimator.Transform = 'projective';
            estimator.AlgebraicDistanceThreshold = obj.maxHomographyError;
            [~, inliersMask] = estimator.step(patternPoints, photoPoints); 
            
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
            
%             figure, showMatchedFeatures(obj.pattern, photo, patternPoints, photoPoints, 'montage');             
            
            
        end
        
        % 
        function obj = calibrate(obj)
            
            obj.initializeCalibration(); 
            obj.optimizeCalibration(); 
            obj.showPoints(); 

            
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

    end
end

        
        
        