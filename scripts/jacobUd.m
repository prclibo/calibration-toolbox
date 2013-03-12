clear
syms r11 r21 r31 r12 r22 r32 r13 r23 r33 t1 t2 t3
syms q11 q21 q31 q12 q22 q32 q13 q23 q33 s1 s2 s3

A = [r11, r12, r13, t1; 
     r21, r22, r23, t2; 
     r31, r32, r33, t3; 
     0, 0, 0, 1]; 

B = [q11, q12, q13, s1; 
     q21, q22, q23, s2; 
     q31, q32, q33, s3; 
     0, 0, 0, 1]; 


AB = A*B; 


return








addpath('utils/')
clear
syms r1 r2 r3 t1 t2 t3

syms gamma1 gamma2 s u0 v0 xi k1 k2 p1 p2

syms u1 u2 sqr_rho

u = [u1; u2]; 

radial_k = k1 * sqr_rho + k2 .* sqr_rho .^2; 
du_radial = u * radial_k; 
du_tangent = [2 * p1 * u(1) * u(2) + p2 * (sqr_rho + 2 * u(1)^2); 
              p1 * (sqr_rho + 2 * u(2)^2) + 2 * p2 * u(1) * u(2)]; 

ud = u + du_radial + du_tangent; 

diff(ud, u1) + diff(ud, sqr_rho) * diff(u1^2 + u2^2, u1)
diff(ud, u2) + diff(ud, sqr_rho) * diff(u1^2 + u2^2, u2)