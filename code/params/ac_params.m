function p = ac_params(opt)
%AC_PARAMS Tham so UAV Aerosonde (nen may bay chinh) va co cau chap hanh.
%   p = AC_PARAMS(opt), opt (tuy chon):
%     alpha0_deg  goc bat dau stall dung trong ham tron (mac dinh 14)
%     dmax_deg    gioi han goc lai elevator/aileron/rudder (mac dinh 25)
%     rate_deg    gioi han toc do lai (mac dinh 200)
%     delay       tre lenh servo (mac dinh 0.07 s)
%
%   Nguon:
%     [R37, App. E, Bang E.1-E.2] Beard & McLain: khoi luong, quan tinh, hinh hoc,
%       he so khi dong, dong co AXI 5345/20 + canh quat APC 20x10E. Bo tham so nay
%       goc tu AeroSim Blockset (Unmanned Dynamics) [R37 dan nguon].
%     Mo hinh luc/momen [R37, Eq. 4.6-4.26]; dong co [R37, Eq. 4.17-4.22].
%   Kiem tra hop ly (xem checks/param_sanity.m) va cac GT:
%     - alpha0 = 0.47 rad (27 deg) trong [R37] cho CL_max ~ 2.6, khong thuc te voi
%       canh UAV nay -> dung alpha0 = 14 deg (GT, quet luoi 12-16 deg).
%     - [R37] khong cho dong hoc/gioi han servo: lay servo bac 2 wn = 100 rad/s,
%       zeta = 0.707, tre 0.07 s [R2, Bang 9] va toc do 200 deg/s [R7] (servo UAV nho);
%       gioi han goc 25 deg (GT, quet luoi 20-30 deg).
%     - rho = 1.225 (ISA muc nuoc bien); [R37] dung 1.268.

if nargin < 1, opt = struct(); end
d = struct('alpha0_deg', 14, 'dmax_deg', 25, 'rate_deg', 200, 'delay', 0.07);
f = fieldnames(d);
for k = 1:numel(f), if ~isfield(opt, f{k}), opt.(f{k}) = d.(f{k}); end, end
d2r = pi/180;

p.name = 'Aerosonde';
p.env.g = 9.81; p.env.rho = 1.225;

%% Hinh hoc, khoi luong [R37, Bang E.1]
p.geom.S = 0.55; p.geom.b = 2.9; p.geom.c = 0.19;
p.geom.AR = p.geom.b^2/p.geom.S; p.geom.e = 0.9;
p.mass.m0 = 11.0;
Jx = 0.824; Jy = 1.135; Jz = 1.759; Jxz = 0.120;
p.mass.J0 = [Jx 0 -Jxz; 0 Jy 0; -Jxz 0 Jz];

%% He so khi dong [R37, Bang E.2]
A = struct( ...
  'CL0', 0.23, 'CD0', 0.0424, 'Cm0', 0.0135, 'CLa', 5.61, 'CDa', 0.132, 'Cma', -2.74, ...
  'CLq', 7.95, 'CDq', 0, 'Cmq', -38.21, 'CLde', 0.13, 'CDde', 0.0135, 'Cmde', -0.99, ...
  'M', 50, 'alpha0', 0.47, 'CDp', 0.043, ...
  'CY0', 0, 'Cl0', 0, 'Cn0', 0, 'CYb', -0.83, 'Clb', -0.13, 'Cnb', 0.073, ...
  'CYp', 0, 'Clp', -0.51, 'Cnp', -0.069, 'CYr', 0, 'Clr', 0.25, 'Cnr', -0.095, ...
  'CYda', 0.075, 'Clda', 0.17, 'Cnda', -0.011, 'CYdr', 0.19, 'Cldr', 0.0024, 'Cndr', -0.069);
A.alpha0_R37 = A.alpha0;
A.alpha0 = opt.alpha0_deg*d2r;          % GT (xem tren)
A.Va_min = 1.0;
p.aero = A;
% Do bat dinh he so cho Monte Carlo: [R37] khong cho do lech chuan -> GT 10 %
% (cung bac voi do lech tuong doi lon nhat trong bang he so nhan dang [R2, Bang 8]).
p.aero_rel_std = 0.10;
p.alpha_lim = (opt.alpha0_deg - 2)*d2r; % bien an toan 2 deg truoc stall (GT)

%% Dong co + canh quat [R37, Bang E.1]
pr = struct('D', 0.508, 'KV', 0.0659, 'KQ', 0.0659, 'R', 0.042, 'i0', 1.5, ...
            'CQ', [0.005230 0.004970 -0.01664], 'CT', [0.09357 -0.06044 -0.1079], ...
            'Vmax', 44.4);
pr.Om_min = 30;                          % GT: duoi muc nay coi canh quat tu do (T = Q = 0)
pr.J_max = 0.9;                          % GT: kep he so tien ngoai vung du lieu
p.prop = pr;

%% Co cau chap hanh: [elevator; aileron; rudder]
p.act.wn = 100; p.act.zeta = 0.707; p.act.delay = opt.delay;
p.act.max = opt.dmax_deg*d2r*[1; 1; 1];
p.act.rate = opt.rate_deg*d2r*[1; 1; 1];
p.act.thr_tau = 0.2; p.act.thr_delay = 0.05;     % ga [R2, Bang 9]

p = ac_apply_config(p, struct('name', {}, 'm', {}, 'r', {}, 'J', {}));
end
