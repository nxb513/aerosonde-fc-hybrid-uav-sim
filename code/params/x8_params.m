function p = x8_params()
%X8_PARAMS Tham so Skywalker X8 lay tu tai lieu da doc toan van.
%   p = X8_PARAMS() tra ve struct tham so danh dinh (cau hinh goc, chua lap
%   FC/pin/sieu tu). Moi so lieu ghi ro nguon; gia thiet cua ta ghi "GT".
%
%   Nguon:
%     [R2] Low-Hansen et al. 2025, CEAS Aeronaut. J., doi:10.1007/s13272-025-00816-3
%          Bang 1 (khoi luong, quan tinh, hinh hoc), Bang 6 (canh quat),
%          Bang 7 (dong co), Bang 8 (he so khi dong + do lech chuan), Bang 9 (servo).
%     [R7] Bohn et al. 2019, ICUAS: gioi han elevon +-30 deg, +-200 deg/s.
%     [R1] Reinhardt et al. 2022, ICUAS: vung tuyen tinh alpha in [-10, 12] deg.

d2r = pi/180;

%% Moi truong
p.env.g   = 9.81;
p.env.rho = 1.225;          % GT: ISA muc nuoc bien; [R2] khong neu rho

%% Hinh hoc [R2, Bang 1]
p.geom.S = 0.75;            % m^2
p.geom.b = 2.10;            % m
p.geom.c = 0.36;            % m (mean aerodynamic chord)

%% Khoi luong va quan tinh danh dinh [R2, Bang 1]
% Goc he than = tam khi dong = CG danh dinh [R2, muc 2.1].
Ix = 0.325; Iy = 0.140; Iz = 0.400; Ixz = 0.029;
p.mass.m0 = 3.364;                              % kg
p.mass.J0 = [Ix 0 -Ixz; 0 Iy 0; -Ixz 0 Iz];     % kg m^2, quanh CG danh dinh [R2, Eq. 24]

%% He so khi dong [R2, Bang 8] -- gia tri va do lech chuan (dung cho Monte Carlo)
% Mo hinh [R2, Eq. 17-18]; goc tinh bang rad; p*,q*,r* khong thu nguyen.
coef = {
%   ten      gia tri     std
    'CD0',   0.058,      6.73e-4
    'CDq',   0.480,      5.85e-2
    'CDCT', -0.217,      1.99e-2
    'CDk1', -0.034,      2.18e-3
    'CDk2',  0.225,      2.33e-3
    'CL0',  -0.077,      3.04e-3
    'CLa',   2.573,      2.33e-2
    'CLq',  17.119,      2.30e-1
    'CLde',  1.369,      2.03e-2
    'CY0',   0.011,      1.91e-4
    'CYb',  -0.285,      3.08e-3
    'CYp',  -0.270,      7.67e-3
    'CYr',   0.108,      1.20e-2
    'CYda',  0.097,      2.24e-3
    'Cl0',   0.007,      7.21e-5
    'Clb',  -0.108,      1.04e-3
    'Clp',  -0.313,      2.91e-3
    'Clr',   0.037,      3.60e-3
    'Clda',  0.102,      8.51e-4
    'Cm0',   0.027,      2.81e-4
    'Cma',  -0.274,      1.97e-3
    'Cmq',  -1.608,      2.44e-2
    'Cmde', -0.276,      1.89e-3
    'Cn0',  -6.3e-4,     9.51e-6
    'Cnb',   0.022,      1.61e-4
    'Cnp',  -0.009,      5.27e-4
    'Cnr',  -0.050,      5.34e-4
    'Cnda', -0.007,      1.48e-4
};
for k = 1:size(coef, 1)
    p.aero.(coef{k,1})     = coef{k,2};
    p.aero_std.(coef{k,1}) = coef{k,3};
end
p.aero.alpha_valid = [-10 12]*d2r;   % [R1]: vung tuyen tinh, stall ~12 deg
p.aero.beta_valid  = [-5 5]*d2r;     % [R1]
p.aero.Va_min      = 1.0;            % GT: chan duoi Va de tranh chia cho 0

%% Canh quat va dong co [R2, Bang 6-7, Eq. 5-9, 28-29]
p.prop.D   = 0.3556;                           % m (14 in)
p.prop.CT  = [0.1400 -0.0300 -0.2370 0.0847];  % CT0..CT3 (bac 3 theo J)
p.prop.CQ  = [0.0082  0.0112 -0.0211];         % CQ0..CQ2 (bac 2 theo J)
p.prop.KE  = 0.0157;                           % V/(rad/s)
p.prop.R   = 0.017;                            % Ohm
p.prop.Ip  = 3.46e-4;                          % kg m^2
p.prop.Im0 = 0;                                % A; [R2] bo qua dong khong tai
p.prop.J_max = 1.0;      % GT: kep he so tien J ngoai vung khop da thuc
p.prop.Om_min = 1.0;     % GT: rad/s, chan duoi khi tinh J
p.prop.allow_regen = false;  % GT: ESC khong hoi sinh (Im >= 0)

%% Co cau chap hanh [R2, Bang 9], gioi han [R7]
p.act.elevon.wn    = 100;          % rad/s
p.act.elevon.zeta  = 0.707;
p.act.elevon.delay = 0.07;         % s
p.act.elevon.max   = 30*d2r;       % rad
p.act.elevon.rate  = 200*d2r;      % rad/s
p.act.throttle.tau   = 0.2;        % s
p.act.throttle.delay = 0.05;       % s
p.act.throttle.min   = 0;
p.act.throttle.max   = 1;

%% Bus DC
% [R2] ghi nhan dien ap nguon 15.2-16.4 V trong chuyen bay; GT: lay 15.8 V.
p.bus.U_nom = 15.8;

%% Cau hinh khoi luong (chua lap them gi)
p = x8_apply_config(p, struct('name', {}, 'm', {}, 'r', {}, 'J', {}));
end
