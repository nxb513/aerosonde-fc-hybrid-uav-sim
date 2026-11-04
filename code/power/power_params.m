function pp = power_params(cfg)
%POWER_PARAMS He nguon lai ban chu dong FC/pin/sieu tu cho Aerosonde chuyen doi.
%   pp = POWER_PARAMS(cfg). Truong cfg (mac dinh trong ngoac):
%     P_fc     cong suat dinh muc FC, W (600). Dinh co so voi cong suat hanh trinh
%              400-470 W cua Aerosonde 11-17 kg o 25 m/s (checks/flight_envelope).
%     fc_wkg   cong suat rieng FC + bo dieu khien, W/kg (458, AeroStack 1 kW 2182 g [R3]).
%     m_H2     luong H2 khi nap day, kg (0.1) -- GT.
%     Td       thoi gian dap ung FC dat 95 %, s (10): 1 [R19] .. 10-20 [R24].
%     ramp     gioi han toc do tang dong cua stack 200 W tham chieu, A/s (2) [R25].
%     Np       so nhanh song song pin 13S (1).
%     sc_type  0 khong sieu tu | 2 Maxwell 16 V 58 F 630 g [R3] (2).
%     esr_sc   ESR sieu tu, Ohm (0.01) -- GT.
%     eta_dc   hieu suat DC-DC (0.95) -- GT.
%     f_c      tan so cat co dinh cua EMS loc tan so, Hz (0.1) -- GT (ems_filter).
%
%   Kien truc ban chu dong [R3][R27][R28]: FC va sieu tu qua DC-DC, pin noi thang bus.
%   Mo hinh:
%     FC   duong phan cuc AeroStack 35 cell [R24, Eq. 1], co theo dien tich cell
%          s = P_fc/200; tre kich hoat tau = Td/3 [R19, Eq. 1].
%     H2   binh nen chua 6 % H2 theo khoi luong, cong bo dieu ap 175 g [R35]:
%          khoi luong binh = m_H2/0.06 + 0.175 kg.
%     Pin  cell LiFePO4 3.3 V 2.3 Ah Tremblay [R4, Bang 1], 13S (42.9 V danh dinh);
%          khoi luong 65 Wh/kg (pack 4S 1500 mAh 343 g [R5]) -- GT.
%     Tai nen avionics + nhiem vu 20 W [R25].
%   Gioi han cung (ps_step, khong bao gio vuot):
%     FC: dong trong [Imin, Imax], tang dong <= ramp, cong suat stack <= P_fc.
%     Pin: dong xa <= 10C, dong sac <= 1C (GT), ngat sac khi SoC = 100 %.
%     Sieu tu: nang luong 25-100 % [R3], dong <= Imax (GT).
if nargin < 1, cfg = struct(); end
def = struct('P_fc', 600, 'fc_wkg', 458, 'm_H2', 0.1, 'Td', 10, 'ramp', 2, 'Np', 1, ...
    'sc_type', 2, 'esr_sc', 0.010, 'eta_dc', 0.95, 'f_c', 0.1);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(cfg, f{k}), cfg.(f{k}) = def.(f{k}); end
end
pp.cfg = cfg;

%% Pin nhien lieu
s = cfg.P_fc/200;
fc.ref = struct('V0', 30.48, 'B', 0.4947, 'R', 0.5002, 'm', 1.972e-4, 'n', 0.6826, ...
                'Ncell', 35, 'Imax', 10);   % [R24]
fc.scale = s;
fc.Ncell = 35;
fc.Imax  = fc.ref.Imax*s;
fc.Imin  = 0.05*fc.Imax;                % GT: FC dang chay khong ha duoi 5 % dong cuc dai
fc.P_rated = cfg.P_fc;
fc.tau   = cfg.Td/3;
fc.ramp  = cfg.ramp*s;
fc.eta_F_hi = 0.9; fc.eta_F_lo = 0.4; fc.P_eta_switch = 50*s;   % [R24]
fc.mass  = cfg.P_fc/cfg.fc_wkg;
fc.mass_tank = cfg.m_H2/0.06 + 0.175;  % binh nen 6 % + bo dieu ap 175 g [R35]
fc.m_H2_cap = cfg.m_H2;                 % dung tich H2 khi nap day (kg)
% Suy giam (chi dung de danh gia): sut ap moi cell [R17, Bang 1], k = 1.72 [R16];
% nguong chay khong tai / tai cao 20 % / 80 % P_max theo [R10, Eq. 16-17].
fc.deg = struct('load_uV', 0.4185, 'idle_uV_h', 8.662, 'high_uV_h', 10.00, ...
                'startstop_uV', 13.79, 'k', 1.72, 'idle_frac', 0.2, 'high_frac', 0.8);
% Diem dinh muc o trang thai xac lap: dong I_rated cho cong suat stack = P_fc, va
% dien ap cell tai do (het tuoi tho khi sut 10 % dien ap nay [R16]).
[fc.I_rated, V_rated] = rated_point(fc);
fc.Vcell_rated = V_rated/fc.Ncell;
pp.fc = fc;

%% Pin
bat.cell = struct('E0', 3.366, 'R', 0.01, 'K', 0.0076, 'A', 0.26422, 'B', 26.5487, ...
                  'Q', 2.3, 'Vnom', 3.3);   % [R4, Bang 1]
bat.Ns = 13; bat.Np = cfg.Np;
bat.tau_f = 30;                         % [R4]
bat.Q = bat.cell.Q*bat.Np;
bat.E_Wh = bat.Ns*bat.Np*bat.cell.Vnom*bat.cell.Q;
bat.mass = bat.E_Wh/65;
bat.I_dis_max = 10*bat.Q;               % GT: 10C ([R18] thu den 10C)
bat.I_chg_max = 1*bat.Q;                % GT: 1C
bat.age = struct('c', [0.5 2 6 10], 'B', [31630 21681 12934 15512], ...
                 'Ea0', 31700, 'Ea1', 370.3, 'z', 0.55, 'T', 298.15, 'Ah_ref', 1000);  % [R18]
pp.bat = bat;

%% Sieu tu
if cfg.sc_type == 2
    sc = struct('use', true, 'C', 58, 'Vmax', 16, 'mass', 0.630, 'Imax', 100);   % [R3]; Imax GT
else
    sc = struct('use', false, 'C', 58, 'Vmax', 16, 'mass', 0, 'Imax', 0);
end
sc.ESR = cfg.esr_sc; sc.e_min = 0.25;   % [R3]
sc.E_J = 0.5*sc.C*sc.Vmax^2;
pp.sc = sc;

%% EMS loc tan so [R3] voi f_c co dinh, cong them hieu chinh giu SoC pin va nang luong
% sieu tu (GT). He so chon theo hang so thoi gian hoi phuc:
%   SoC pin:   tau = E_pin/(k_soc P_fc) = 98.7 Wh * 3600/(2 * 600 W) ~ 5 phut
%              (dai hon chan troi 10 s, ngan hon mot nhiem vu dai);
%   sieu tu:   tau = E_sc/(k_e P_fc) = 7.4 kJ/(0.5 * 600 W) ~ 25 s
%              (sieu tu ve lai 50 % truoc dot gio giat tiep theo).
pp.ems = struct('f_c', cfg.f_c, 'k_soc', 2.0, 'k_e', 0.5);

pp.dc.eta = cfg.eta_dc;
pp.aux.P = 20;
pp.mass_total = fc.mass + fc.mass_tank + bat.mass + sc.mass;
pp.F = 96485; pp.M_H2 = 2.016e-3;
end

function [I, V] = rated_point(fc)
% Dong xac lap tai do cong suat stack = P_rated (chia doi tren [Imin, Imax])
lo = fc.Imin; hi = fc.Imax;
P = @(I) stack_power(I, fc);
if P(hi) <= fc.P_rated, I = hi; V = P(hi)/hi; return; end
for k = 1:60
    mid = 0.5*(lo + hi);
    if P(mid) > fc.P_rated, hi = mid; else, lo = mid; end
end
I = lo; V = P(lo)/lo;
end

function P = stack_power(I, fc)
[~, va] = fc_voltage(I, 0, fc);
P = fc_voltage(I, va, fc)*I;
end
