function ap = autopilot_design(p, tr, des)
%AUTOPILOT_DESIGN Do loi autopilot dong vong noi tiep [R37, ch. 6] tu mo hinh tai trim.
%   ap = AUTOPILOT_DESIGN(p, tr, des)
%   He so ham truyen lay tu tuyen tinh hoa so (x8_linearize) thay vi cong thuc giai tich:
%     a_phi1 = -dp/dp, a_phi2 = dp/d(delta_a)                   [R37, Eq. 5.22-5.23]
%     a_th1 = -dq/dq, a_th2 = -Va*dq/dw, a_th3 = dq/d(delta_e) [R37, Eq. 5.27]
%     a_V1 = -du/du, a_V2 = du/d(delta_t)                      [R37, Eq. 5.34]
%   Do loi theo [R37, Eq. 6.1-6.15]; tan so/giam chan la tham so thiet ke (GT, des).
%
%   Vong pitch (sua doi cua ta, GT): cong thuc [R37, Eq. 6.9-6.10] voi wn_th co dinh
%   cho do loi doi dau khi do cung tu nhien a_th2 > wn_th^2 (CG lech mui) va do loi
%   phan hoi qua lon khi a_th2 nho (CG lech duoi) -> mat on dinh voi tre servo 0.07 s
%   [R2, Bang 9]. Do do chon do cung phan hoi
%     K_fb = kp_th*a_th3 = max(wn_th^2 - a_th2, c_min*a_th2) > 0  (luon dung dau, K_DC > 0)
%   va gioi han do loi vong toc do goc |kd_th*a_th3| <= kq_max (bang thong < 1/(2*tre)).

if nargin < 3, des = struct(); end
d = struct('wn_phi', 8, 'z_phi', 0.9, 'W_chi', 8, 'z_chi', 1.0, ...
           'wn_th', 8, 'z_th', 0.7, 'c_min', 0.15, 'kq_max', 5, ...
           'W_h', 10, 'z_h', 1.0, 'wn_V', 1.0, 'z_V', 1.0);
f = fieldnames(d);
for k = 1:numel(f), if ~isfield(des, f{k}), des.(f{k}) = d.(f{k}); end, end

lin = x8_linearize(p, tr);
A = lin.A; B = lin.B;          % xe = [u v w p q r phi theta psi Om], u = [der del dt]
Bde = B(:, 1) + B(:, 2);       % der = de - da, del = de + da
Bda = -B(:, 1) + B(:, 2);
Va = tr.Va; g = p.env.g;

a_phi1 = -A(4, 4); a_phi2 = Bda(4);
a_th1  = -A(5, 5); a_th2 = -Va*A(5, 3); a_th3 = Bde(5);
a_V1   = -A(1, 1); a_V2  = B(1, 3);
% Toc do canh quat lam cham dap ung ga; a_V2 lay theo xac lap Om (GT):
dOm_ddt = -A(10, 10)\B(10, 3);
a_V2 = a_V2 + A(1, 10)*dOm_ddt;

ap.kp_phi = des.wn_phi^2/a_phi2;
ap.kd_phi = (2*des.z_phi*des.wn_phi - a_phi1)/a_phi2;
wn_chi = des.wn_phi/des.W_chi;
ap.kp_chi = 2*des.z_chi*wn_chi*Va/g;
ap.ki_chi = wn_chi^2*Va/g;
K_fb = max(des.wn_th^2 - max(a_th2, 0), des.c_min*max(a_th2, 1));
wn_cl = sqrt(max(a_th2, 0) + K_fb);
ap.kp_th = K_fb/a_th3;
kq = min(max(2*des.z_th*wn_cl - a_th1, 0), des.kq_max);
ap.kd_th = kq/a_th3;
K_DC = K_fb/wn_cl^2;
wn_h = wn_cl/des.W_h;
ap.ki_h = wn_h^2/(K_DC*Va);
ap.kp_h = 2*des.z_h*wn_h/(K_DC*Va);
ap.ki_V = des.wn_V^2/a_V2;
ap.kp_V = (2*des.z_V*des.wn_V - a_V1)/a_V2;

ap.trim = struct('de', tr.de, 'da', tr.da, 'dt', tr.dt, 'theta', tr.theta, 'Va', Va);
ap.theta_lim = [-15 25]*pi/180;     % GT
ap.coef = struct('a_phi1', a_phi1, 'a_phi2', a_phi2, 'a_th1', a_th1, 'a_th2', a_th2, ...
                 'a_th3', a_th3, 'a_V1', a_V1, 'a_V2', a_V2, 'K_DC', K_DC, 'wn_th_cl', wn_cl);
end
