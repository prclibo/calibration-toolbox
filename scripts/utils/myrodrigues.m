function [dst, jacobian] = rodrigues(src)
if numel(src) == 3 && norm(src) ~= 0

    theta = norm(src); 
    cos_theta = cos(theta); 
    sin_theta = sin(theta); 
    r1 = src(1); 
    r2 = src(2); 
    r3 = src(3); 
    
    R = vrrotvec2mat([src(:) ./ theta; theta]); 
    dst = R; 
    if nargout < 2
        return; 
    end
    
    pR_dr1 = [                                      (r1*((2*r1^2*(cos_theta - 1))/theta^3 - sin_theta + (r1^2*sin_theta)/theta^2))/theta - (2*r1*(cos_theta - 1))/theta^2, (r1*((r3*sin_theta)/theta^2 - (r3*cos_theta)/theta + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta - (r2*(cos_theta - 1))/theta^2, (r1*((r2*cos_theta)/theta - (r2*sin_theta)/theta^2 + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta - (r3*(cos_theta - 1))/theta^2; 
              (r1*((r3*cos_theta)/theta - (r3*sin_theta)/theta^2 + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta - (r2*(cos_theta - 1))/theta^2,                                                                        (r1*((2*r2^2*(cos_theta - 1))/theta^3 - sin_theta + (r2^2*sin_theta)/theta^2))/theta,              (r1*((r1*sin_theta)/theta^2 - (r1*cos_theta)/theta + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta - sin_theta/theta; 
              (r1*((r2*sin_theta)/theta^2 - (r2*cos_theta)/theta + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta - (r3*(cos_theta - 1))/theta^2,              sin_theta/theta + (r1*((r1*cos_theta)/theta - (r1*sin_theta)/theta^2 + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta,                                                                        (r1*((2*r3^2*(cos_theta - 1))/theta^3 - sin_theta + (r3^2*sin_theta)/theta^2))/theta]; 

    pR_dr2 = [                                                                        (r2*((2*r1^2*(cos_theta - 1))/theta^3 - sin_theta + (r1^2*sin_theta)/theta^2))/theta, (r2*((r3*sin_theta)/theta^2 - (r3*cos_theta)/theta + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta - (r1*(cos_theta - 1))/theta^2,              sin_theta/theta + (r2*((r2*cos_theta)/theta - (r2*sin_theta)/theta^2 + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta; 
              (r2*((r3*cos_theta)/theta - (r3*sin_theta)/theta^2 + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta - (r1*(cos_theta - 1))/theta^2,                                      (r2*((2*r2^2*(cos_theta - 1))/theta^3 - sin_theta + (r2^2*sin_theta)/theta^2))/theta - (2*r2*(cos_theta - 1))/theta^2, (r2*((r1*sin_theta)/theta^2 - (r1*cos_theta)/theta + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta - (r3*(cos_theta - 1))/theta^2; 
              (r2*((r2*sin_theta)/theta^2 - (r2*cos_theta)/theta + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta - sin_theta/theta, (r2*((r1*cos_theta)/theta - (r1*sin_theta)/theta^2 + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta - (r3*(cos_theta - 1))/theta^2,                                                                        (r2*((2*r3^2*(cos_theta - 1))/theta^3 - sin_theta + (r3^2*sin_theta)/theta^2))/theta]; 

    pR_dr3 = [                                                                        (r3*((2*r1^2*(cos_theta - 1))/theta^3 - sin_theta + (r1^2*sin_theta)/theta^2))/theta,              (r3*((r3*sin_theta)/theta^2 - (r3*cos_theta)/theta + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta - sin_theta/theta, (r3*((r2*cos_theta)/theta - (r2*sin_theta)/theta^2 + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta - (r1*(cos_theta - 1))/theta^2; 
               sin_theta/theta + (r3*((r3*cos_theta)/theta - (r3*sin_theta)/theta^2 + (2*r1*r2*(cos_theta - 1))/theta^3 + (r1*r2*sin_theta)/theta^2))/theta,                                                                        (r3*((2*r2^2*(cos_theta - 1))/theta^3 - sin_theta + (r2^2*sin_theta)/theta^2))/theta, (r3*((r1*sin_theta)/theta^2 - (r1*cos_theta)/theta + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta - (r2*(cos_theta - 1))/theta^2; 
               (r3*((r2*sin_theta)/theta^2 - (r2*cos_theta)/theta + (2*r1*r3*(cos_theta - 1))/theta^3 + (r1*r3*sin_theta)/theta^2))/theta - (r1*(cos_theta - 1))/theta^2, (r3*((r1*cos_theta)/theta - (r1*sin_theta)/theta^2 + (2*r2*r3*(cos_theta - 1))/theta^3 + (r2*r3*sin_theta)/theta^2))/theta - (r2*(cos_theta - 1))/theta^2,                                      (r3*((2*r3^2*(cos_theta - 1))/theta^3 - sin_theta + (r3^2*sin_theta)/theta^2))/theta - (2*r3*(cos_theta - 1))/theta^2]; 
 
    
    jacobian(:, 1) = pR_dr1(:); 
    jacobian(:, 2) = pR_dr2(:); 
    jacobian(:, 3) = pR_dr3(:);     

elseif numel(src) == 3 && norm(src) == 0
    dst = eye(3);
    
    if nargout < 2
        return; 
    end
    
    jacobian = [    0.0000   -0.0000   -0.0000
                    0.0000    0.0000    1.0000
                    0.0000   -1.0000    0.0000
                    0.0000    0.0000   -1.0000
                   -0.0000    0.0000   -0.0000
                    1.0000    0.0000    0.0000
                    0.0000    1.0000    0.0000
                   -1.0000    0.0000    0.0000
                   -0.0000   -0.0000    0.0000];     
else
    dst = vrrotmat2vec(src);
    dst = dst(1:3) .* dst(4);
    jacobian = []; 
end
end