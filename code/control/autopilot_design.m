function ap = autopilot_design(p, tr, des)
%AUTOPILOT_DESIGN Autopilot dong vong noi tiep [R37, ch. 6] cho Aerosonde, do loi tu
%   mo hinh tuyen tinh tai trim (ac_linearize).
%   He so ham truyen [R37, Eq. 5.22-5.35]:
%     a_phi1 = -Lp, a_phi2 = L_da; a_th1 = -Mq, a_th2 = -Va*Mw, a_th3 = M_de;
%     a_V1 = -Xu, a_V2 = X_dt.
%   Roll, course, altitude, airspeed theo [R37, Eq. 6.1-6.15]; yaw damper co bo loc
%   washout p_wo = w_dr/10 va k_r cho zeta = 0.707 [R37, muc 6.1.1, Eq. 6.7].
%   Sua doi cua ta (GT), de ben voi dich CG va tre servo:
%     pitch: do cung phan hoi K_fb = max(wn_th^2 - a_th2, c_min*a_th2) > 0 (luon dung
%            dau); gioi han do loi vong toc do |kd*a_th3| <= kq_max.
%     roll:  kd_phi >= 0.
if nargin < 3, des = struct(); end
d = struct('wn_phi', 8, 'z_phi', 0.9, 'W_chi', 8, 'z_chi', 1.0, ...
           'wn_th', 8, 'z_th', 0.7, 'c_min', 0.15, 'kq_max', 5, ...
           'W_h', 10, 'z_h', 1.0, 'wn_V', 0.8, 'z_V', 1.0);
f = fieldnames(d);
for k = 1:numel(f), if ~isfield(des, f{k}), des.(f{k}) = d.(f{k}); end, end

lin = ac_linearize(p, tr);
A = lin.A; B = lin.B;          % xe = [u v w p q r phi theta psi], u = [de da dr dt]
Va = tr.Va; g = p.env.g;
a_phi1 = -A(4, 4); a_phi2 = B(4, 2);
a_th1 = -A(5, 5); a_th2 = -Va*A(5, 3); a_th3 = B(5, 1);
a_V1 = -A(1, 1); a_V2 = B(1, 4);

ap.kp_phi = des.wn_phi^2/a_phi2;
ap.kd_phi = max(2*des.z_phi*des.wn_phi - a_phi1, 0)/a_phi2;
wn_chi = des.wn_phi/des.W_chi;
ap.kp_chi = 2*des.z_chi*wn_chi*Va/g;
ap.ki_chi = wn_chi^2*Va/g;

K_fb = max(des.wn_th^2 - max(a_th2, 0), des.c_min*max(a_th2, 1));
wn_cl = sqrt(max(a_th2, 0) + K_fb);
ap.kp_th = K_fb/a_th3;
ap.kd_th = min(max(2*des.z_th*wn_cl - a_th1, 0), des.kq_max)/a_th3;
K_DC = K_fb/wn_cl^2;
wn_h = wn_cl/des.W_h;
ap.ki_h = wn_h^2/(K_DC*Va);
ap.kp_h = 2*des.z_h*wn_h/(K_DC*Va);
ap.ki_V = des.wn_V^2/a_V2;
ap.kp_V = max(2*des.z_V*des.wn_V - a_V1, 0)/a_V2;

% Yaw damper (dang beta cua [R37, Eq. 5.55])
Yv = A(2, 2); Yr = A(2, 6)/Va; Nv = A(6, 2)*Va; Nr = A(6, 6);
Ydr = B(2, 3)/Va; Ndr = B(6, 3);
w_dr = sqrt(max(Yv*Nr - Yr*Nv, 0));
ap.p_wo = w_dr/10;
rr = roots([Ndr^2, 2*(Nr*Ndr + Ydr*Nv), Yv^2 + Nr^2 + 2*Yr*Nv]);
rr = real(rr(abs(imag(rr)) < 1e-9 & real(rr) > 0));
if isempty(rr), ap.k_r = 0; else, ap.k_r = min(rr); end

ap.trim = struct('de', tr.de, 'da', tr.da, 'dr', tr.dr, 'dt', tr.dt, 'theta', tr.theta, 'Va', Va);
ap.theta_lim = [-15 20]*pi/180;     % GT
ap.coef = struct('a_phi1', a_phi1, 'a_phi2', a_phi2, 'a_th1', a_th1, 'a_th2', a_th2, ...
                 'a_th3', a_th3, 'a_V1', a_V1, 'a_V2', a_V2, 'K_DC', K_DC, 'wn_th_cl', wn_cl, ...
                 'w_dr', w_dr);
end
