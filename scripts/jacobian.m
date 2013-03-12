addpath('utils/')
clear
syms r1 r2 r3 t1 t2 t3

t = [t1; t2; t3]; 
% theta = sqrt(r1^2 + r2^2 + r3^2); 
% r = [r1; r2; r3] / theta; 
% 
% R = cos(theta) + (1 - cos(theta)) * (r * transpose(r)) + sin(theta) * skew(r); 

syms r11 r21 r31 r12 r22 r32 r13 r23 r33
R = [r11, r12, r13; r21, r22, r23; r31, r32, r33]; 
syms gamma1 gamma2 s u0 v0 xi k1 k2 p1 p2
syms X Y Z x y L

P = R * [X; Y; Z] + t; 
% P = [X; Y; Z]; 
Lexpd = sqrt(P(1)^2 + P(2)^2 + P(3)^2); 
P = P / L; 
P(3) = P(3) + xi; 

u = P(1:2) ./ P(3);

pp_r11 = diff(u, r11) + diff(u, L) * diff(Lexpd, r11)
pp_r21 = diff(u, r21) + diff(u, L) * diff(Lexpd, r21)
pp_r31 = diff(u, r31) + diff(u, L) * diff(Lexpd, r31) 
pp_r12 = diff(u, r12)  + diff(u, L) * diff(Lexpd, r12)
pp_r22 = diff(u, r22)  + diff(u, L) * diff(Lexpd, r22)
pp_r32 = diff(u, r32)  + diff(u, L) * diff(Lexpd, r32)
pp_r13 = diff(u, r13)  + diff(u, L) * diff(Lexpd, r13)
pp_r23 = diff(u, r23)  + diff(u, L) * diff(Lexpd, r23)
pp_r33 = diff(u, r33)  + diff(u, L) * diff(Lexpd, r33)
pp_t1 = diff(u, t1)  + diff(u, L) * diff(Lexpd, t1)
pp_t2 = diff(u, t2)  + diff(u, L) * diff(Lexpd, t2)
pp_t3 = diff(u, t3)  + diff(u, L) * diff(Lexpd, t3)

return
syms sqr_rho
% sqr_rho = u(1)^2 + u(2)^2; 

% syms u1 u2 sqr_rho
% u = [u1; u2]; 

% radial_k = k1 * sqr_rho + k2 .* sqr_rho .^2; 
% du_radial = u * radial_k; 
% du_tangent = [2 * p1 * u(1) * u(2) + p2 * (sqr_rho + 2 * u(1)^2); 
%               p1 * (sqr_rho + 2 * u(2)^2) + 2 * p2 * u(1) * u(2)]; 
% 
% ud = u + du_radial + du_tangent; 
% diff(ud, xi) + diff(ud, sqr_rho) * diff(u(1)^2 + u(2)^2, xi)


syms ud1 ud2
ud = [ud1; ud2; 1]; 
K = [gamma1, s, u0; 
     0, gamma2, v0; 
     0, 0, 1]; 
p = K * ud; 

diff(p, v0)

% diff(p, u1) + diff(p, sqr_rho) * diff(u1^2 + u2^2, u1)

% diff(p(2), xi)

% pp_g1 = diff(p, gamma1)
% pp_g2 = diff(p, gamma2)
% pp_s = diff(p, s) 
% pp_u0 = diff(p, u0) 
% pp_v0 = diff(p, v0) 
% pp_k1 = diff(p, k1) 
% pp_k2 = diff(p, k2) 
% pp_p1 = diff(p, p1) 
% pp_p2 = diff(p, p2)


