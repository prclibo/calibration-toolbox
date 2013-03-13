classdef CataCamera < handle
    properties
        width
        height
        
        nParams = 10; 
        
        gamma1
        gamma2
        s
        u0
        v0
        
        xi
        k1
        k2
        p1
        p2
    end
    
    methods
        % 
        function cataCamera = CataCamera(width, height, gamma1, gamma2, s, u0, v0, xi, k1, k2, p1, p2)
            cataCamera.width = width; 
            cataCamera.height = height; 
            
            if (nargin <= 2) 
                return; 
            end
            
            cataCamera.gamma1 = gamma1; 
            cataCamera.gamma2 = gamma2; 
            cataCamera.s = s; 
            cataCamera.u0 = u0; 
            cataCamera.v0 = v0; 
            cataCamera.xi = xi; 
            cataCamera.k1 = k1; 
            cataCamera.k2 = k2; 
            cataCamera.p1 = p1; 
            cataCamera.p2 = p2; 
        end
        
        function p = toParamVector(cataCamera)
            p = [cataCamera.gamma1; cataCamera.gamma2; cataCamera.s; 
                 cataCamera.u0; cataCamera.v0; cataCamera.xi; 
                 cataCamera.k1; cataCamera.k2; cataCamera.p1; cataCamera.p2]; 
        end
        
        function cataCamera = fromParamVector(cataCamera, p)
            cataCamera.gamma1 = p(1);
            cataCamera.gamma2 = p(2); 
            cataCamera.s = 0; 
            cataCamera.u0 = p(4); 
            cataCamera.v0 = p(5); 
            cataCamera.xi = p(6); 
            cataCamera.k1 = p(7); 
            cataCamera.k2 = p(8); 
            cataCamera.p1 = p(9); 
            cataCamera.p2 = p(10); 
        end
        
        function [x, jacobPose, jacobCamera] = projectPoints(cataCamera, points, rvec, tvec)
            [R, pR_dr] = rodrigues(rvec); 
            t = tvec; 
            
            gamma1 = cataCamera.gamma1; 
            gamma2 = cataCamera.gamma2; 
            s = cataCamera.s; 
            u0 = cataCamera.u0; 
            v0 = cataCamera.v0; 
            xi = cataCamera.xi; 
            k1 = cataCamera.k1; 
            k2 = cataCamera.k2; 
            p1 = cataCamera.p1; 
            p2 = cataCamera.p2; 
            
            X = points; 
            Xc = R * X + repmat(t, 1, size(X, 2)); 
            Xs = Xc; 
            L = sqrt(sum(Xs.^2)); 
            Xs = Xs ./ repmat(L, 3, 1);             
            X_lift = Xs; 
            X_lift(3, :) = X_lift(3, :) + xi; 
            
            u = X_lift ./ repmat(X_lift(3, :), 3, 1); 
            u = u(1:2, :); 
            
            sqr_rho = u(1, :).^2 + u(2, :).^2; 
            qua_rho = sqr_rho .* sqr_rho; 
            
            radial_k = k1 .* sqr_rho + k2 .* qua_rho; 
            du_radial = u .* repmat(radial_k, 2, 1) ; 
            du_tangent = [2 .* p1 .* u(1, :) .* u(2, :) + p2 .* (sqr_rho + 2 * u(1, :).^2); 
                          p1 .* (sqr_rho + 2 * u(2, :).^2) + 2 * p2 .* u(1, :) .* u(2, :)]; 
            
            ud = u + du_radial + du_tangent; 
            
            K = [gamma1, s, u0; 
                 0, gamma2, v0; 
                 0, 0, 1]; 
            ud(3, :) = 1; 
            x = K * ud; 
            x = x(1:2, :); 

            %% Jacobian: 
            if (nargout < 2) 
                return; 
            end
            % Some temp variables
            X_lift_sqr = X_lift .* X_lift;
            X_lift_cub = X_lift_sqr .* X_lift; 

            temp1 = k1 + 2.*k2.*sqr_rho;
            temp2 = k2.*qua_rho + k1.*sqr_rho;
            temp3 = L.*X_lift(3, :); 
            temp4 = L.^2.*X_lift(3, :); 
            temp5 = L.^2.*X_lift_sqr(3, :); 
            temp6 = L.^3.*X_lift_sqr(3, :); 
            temp7 = Xc(1, :)./(temp4); 
            temp8 = Xc(2, :)./(temp4); 
            temp9 = (Xc(1, :).*Xc(3, :)); 
            temp10 = (Xc(2, :).*Xc(3, :)); 
            temp11 = temp8 - temp10./(temp6); 
            temp12 = temp7 - temp9./(temp6); 
            
            Lc = (Xc(1, :).^2 + Xc(2, :).^2 + Xc(3, :).^2).^(1./2); 

            
            
            px1_dud = repmat([gamma1; 0], 1, size(points, 2)); 
            px2_dud = repmat([s; gamma2], 1, size(points, 2)); 

            pud_du1(1, :) = k1.*sqr_rho + 2.*p1.*u(2, :) + 4.*p2.*u(1, :) + 2.*u(1, :).*(p2 + u(1, :).*(temp1)) + k2.*qua_rho + 1; 
            pud_du1(2, :) = 2.*p2.*u(2, :) + 2.*u(1, :).*(p1 + u(2, :).*(temp1)); 
 
            pud_du2(1, :) = 2.*p1.*u(1, :) + 2.*u(2, :).*(p2 + u(1, :).*(temp1)); 
            pud_du2(2, :) = k1.*sqr_rho + 4.*p1.*u(2, :) + 2.*p2.*u(1, :) + 2.*u(2, :).*(p1 + u(2, :).*(temp1)) + k2.*qua_rho + 1; 
            
            px1_du = [sum(px1_dud .* pud_du1); sum(px1_dud .* pud_du2)]; 
            px2_du = [sum(px2_dud .* pud_du1); sum(px2_dud .* pud_du2)]; 

 
            pu_dr11(1, :) = X(1, :)./(temp3) - (X(1, :).*(temp12).*Xc(1, :))./Lc; 
            pu_dr11(2, :) = -(X(1, :).*(temp11).*Xc(1, :))./Lc; 
            px_dr11 = [sum(px1_du .* pu_dr11); sum(px2_du .* pu_dr11)]; 
            
            pu_dr21(1, :) = -(X(1, :).*(temp12).*Xc(2, :))./Lc; 
            pu_dr21(2, :) = X(1, :)./(temp3) - (X(1, :).*(temp11).*Xc(2, :))./Lc; 
            px_dr21 = [sum(px1_du .* pu_dr21); sum(px2_du .* pu_dr21)]; 
            
            pu_dr31(1, :) = - (X(1, :).*Xc(1, :))./(temp5) - (X(1, :).*(temp12).*Xc(3, :))./Lc; 
            pu_dr31(2, :) = - (X(1, :).*Xc(2, :))./(temp5) - (X(1, :).*(temp11).*Xc(3, :))./Lc; 
            px_dr31 = [sum(px1_du .* pu_dr31); sum(px2_du .* pu_dr31)]; 
            
            pu_dr12(1, :) = X(2, :)./(temp3) - (X(2, :).*(temp12).*Xc(1, :))./Lc; 
            pu_dr12(2, :) = -(X(2, :).*(temp11).*Xc(1, :))./Lc; 
            px_dr12 = [sum(px1_du .* pu_dr12); sum(px2_du .* pu_dr12)]; 
            
            pu_dr22(1, :) = -(X(2, :).*(temp12).*Xc(2, :))./Lc;
            pu_dr22(2, :) = X(2, :)./(temp3) - (X(2, :).*(temp11).*Xc(2, :))./Lc; 
            px_dr22 = [sum(px1_du .* pu_dr22); sum(px2_du .* pu_dr22)]; 
            
            pu_dr32(1, :) = - (X(2, :).*Xc(1, :))./(temp5) - (X(2, :).*(temp12).*Xc(3, :))./Lc; 
            pu_dr32(2, :) = - (X(2, :).*Xc(2, :))./(temp5) - (X(2, :).*(temp11).*Xc(3, :))./Lc; 
            px_dr32 = [sum(px1_du .* pu_dr32); sum(px2_du .* pu_dr32)]; 
            
            pu_dr13(1, :) = X(3, :)./(temp3) - (X(3, :).*(temp12).*Xc(1, :))./Lc; 
            pu_dr13(2, :) = -(X(3, :).*(temp11).*Xc(1, :))./Lc; 
            px_dr13 = [sum(px1_du .* pu_dr13); sum(px2_du .* pu_dr13)]; 
            
            pu_dr23(1, :) = -(X(3, :).*(temp12).*Xc(2, :))./Lc; 
            pu_dr23(2, :) = X(3, :)./(temp3) - (X(3, :).*(temp11).*Xc(2, :))./Lc; 
            px_dr23 = [sum(px1_du .* pu_dr23); sum(px2_du .* pu_dr23)]; 
            
            pu_dr33(1, :) = - (X(3, :).*Xc(1, :))./(temp5) - (X(3, :).*(temp12).*Xc(3, :))./Lc; 
            pu_dr33(2, :) = - (X(3, :).*Xc(2, :))./(temp5) - (X(3, :).*(temp11).*Xc(3, :))./Lc; 
            px_dr33 = [sum(px1_du .* pu_dr33); sum(px2_du .* pu_dr33)]; 

            pu_dt1(1, :) = 1./(temp3) - ((temp12).*(2.*Xc(1, :)))./(2.*Lc); 
            pu_dt1(2, :) = -((temp11).*(2.*Xc(1, :)))./(2.*Lc); 
            px_dt1 = [sum(px1_du .* pu_dt1); sum(px2_du .* pu_dt1)];

            pu_dt2(1, :) = -((temp12).*(2.*Xc(2, :)))./(2.*Lc); 
            pu_dt2(2, :) = 1./(temp3) - ((temp11).*(2.*Xc(2, :)))./(2.*Lc); 
            px_dt2 = [sum(px1_du .* pu_dt2); sum(px2_du .* pu_dt2)]; 
            
            pu_dt3(1, :) = - ((temp12).*(2.*Xc(3, :)))./(2.*Lc) - Xc(1, :)./(temp5); 
            pu_dt3(2, :) = - ((temp11).*(2.*Xc(3, :)))./(2.*Lc) - Xc(2, :)./(temp5); 
            px_dt3 = [sum(px1_du .* pu_dt3); sum(px2_du .* pu_dt3)]; 
            
            
            jacobPose = [px_dr11(:), px_dr21(:), px_dr31(:), px_dr12(:), px_dr22(:), px_dr32(:), px_dr13(:), px_dr23(:), px_dr33(:)]; 
            jacobPose = jacobPose * pR_dr; 
            jacobPose = [jacobPose, px_dt1(:), px_dt2(:), px_dt3(:)]; 
            
            if (nargout < 3)
                return; 
            end
            
            px_dgamma1(1, :) = ud(1, :);
            px_dgamma1(2, :) = 0; 
            px_dgamma2(2, :) = ud(2, :);
            px_dgamma2(1, :) = 0; 
            
            px_ds(1, :) = ud(2, :); 
            px_ds(2, :) = 0; 
            
            px_du0 = repmat([1; 0], 1, size(points, 2)); 
            px_dv0 = repmat([0; 1], 1, size(points, 2)); 

            
            pud_dxi(1, :) = - (p2 + (X_lift(1, :).*(temp1))./X_lift(3, :)).*((2.*X_lift_sqr(1, :))./X_lift_cub(3, :) + (2.*X_lift_sqr(2, :))./X_lift_cub(3, :)) - X_lift(1, :)./X_lift_sqr(3, :) - (4.*X_lift_sqr(1, :).*p2)./X_lift_cub(3, :) - (X_lift(1, :).*(temp2))./X_lift_sqr(3, :) - (4.*X_lift(1, :).*X_lift(2, :).*p1)./X_lift_cub(3, :); 
            pud_dxi(2, :) = - (p1 + (X_lift(2, :).*(temp1))./X_lift(3, :)).*((2.*X_lift_sqr(1, :))./X_lift_cub(3, :) + (2.*X_lift_sqr(2, :))./X_lift_cub(3, :)) - X_lift(2, :)./X_lift_sqr(3, :) - (4.*X_lift_sqr(2, :).*p1)./X_lift_cub(3, :) - (X_lift(2, :).*(temp2))./X_lift_sqr(3, :) - (4.*X_lift(1, :).*X_lift(2, :).*p2)./X_lift_cub(3, :);             
            px_dxi = [sum(px1_dud .* pud_dxi); sum(px2_dud .* pud_dxi)]; 
            
            pud_dk1(1, :) = sqr_rho .* u(1, :); 
            pud_dk1(2, :) = sqr_rho .* u(2, :); 
            px_dk1 = [sum(px1_dud .* pud_dk1); sum(px2_dud .* pud_dk1)]; 
            
            pud_dk2(1, :) = qua_rho .* u(1, :); 
            pud_dk2(2, :) = qua_rho .* u(2, :); 
            px_dk2 = [sum(px1_dud .* pud_dk2); sum(px2_dud .* pud_dk2)]; 
            
            pud_dp1(1, :) = 2 .* u(1, :) .* u(2, :); 
            pud_dp1(2, :) = 2 .* u(2, :).^2 + sqr_rho; 
            px_dp1 = [sum(px1_dud .* pud_dp1); sum(px2_dud .* pud_dp1)]; 
            
            pud_dp2(1, :) = 2 .* u(1, :).^2 + sqr_rho; 
            pud_dp2(2, :) = 2. * u(1, :) .* u(2, :); 
            px_dp2 = [sum(px1_dud .* pud_dp2); sum(px2_dud .* pud_dp2)]; 
                        
            jacobCamera = [px_dgamma1(:), px_dgamma2(:), px_ds(:), px_du0(:), px_dv0(:), px_dxi(:), px_dk1(:), px_dk2(:), px_dp1(:), px_dp2(:)]; 
            
            

            
        end
        
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
        function outputIntrinsics(obj)
            display(['....xi: [', num2str(obj.xi), ']']); 
            display(['....Focal length: [', num2str(obj.gamma1), ', ', num2str(obj.gamma2), ']'])
            display(['....Aspect ratio: ', num2str(obj.s)]); 
            display(['....Principle Point: [', num2str(obj.u0), ', ', num2str(obj.v0), ']']); 
            display(['....Distortion Coeff: [', num2str(obj.k1), ', ', num2str(obj.k2), ', ', num2str(obj.p1), ', ', num2str(obj.p2), ']']);                         
        end

    end
end
