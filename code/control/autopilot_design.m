function ap = autopilot_design(p, tr, des)
%AUTOPILOT_DESIGN Do loi autopilot dong vong noi tiep [R37, ch. 6] tu mo hinh tai trim.
%   ap = AUTOPILOT_DESIGN(p, tr, des)
%   He so ham truyen lay tu tuyen tinh hoa so (x8_linearize) thay vi cong thuc giai tich:
%     a_phi1 = -dp/dp, a_phi2 = dp/d(delta_a)                   [R37, Eq. 5.22-5.23]
%     a_th1 = -dq/dq, a_th2 = -Va*dq/dw, a_th3 = dq/d(delta_e) [R37, Eq. 5.27]
%     a_V1 = -du/du, a_V2 = du/d(delta_t)                      [R37, Eq. 5.34]
%   Do loi theo [R37, Eq. 6.1-6.15]; tan so/giam chan la tham so thiet ke (GT, des).

if nargin < 3, des = struct(); end
d = struct('wn_phi', 10, 'z_phi', 0.9, 'W_chi', 8, 'z_chi', 1.0, ...
           'wn_th', 12, 'z_th', 0.8, 'W_h', 15, 'z_h', 1.0, 'wn_V', 1.0, 'z_V', 1.0);
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
ap.kp_th = (des.wn_th^2 - a_th2)/a_th3;
ap.kd_th = (2*des.z_th*des.wn_th - a_th1)/a_th3;
K_DC = ap.kp_th*a_th3/des.wn_th^2;
wn_h = des.wn_th/des.W_h;
ap.ki_h = wn_h^2/(K_DC*Va);
ap.kp_h = 2*des.z_h*wn_h/(K_DC*Va);
ap.ki_V = des.wn_V^2/a_V2;
ap.kp_V = (2*des.z_V*des.wn_V - a_V1)/a_V2;

ap.trim = struct('de', tr.de, 'da', tr.da, 'dt', tr.dt, 'theta', tr.theta, 'Va', Va);
ap.theta_lim = [-15 25]*pi/180;     % GT
ap.coef = struct('a_phi1', a_phi1, 'a_phi2', a_phi2, 'a_th1', a_th1, 'a_th2', a_th2, ...
                 'a_th3', a_th3, 'a_V1', a_V1, 'a_V2', a_V2, 'K_DC', K_DC);
end
