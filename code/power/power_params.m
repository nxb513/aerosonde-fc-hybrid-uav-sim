function pp = power_params(cfg)
%POWER_PARAMS He nguon lai FC/pin/sieu tu cho Aerosonde chuyen doi.
%   pp = POWER_PARAMS(cfg). Truong cfg (mac dinh trong ngoac):
%     P_fc     cong suat dinh muc FC, W (600). Dinh co so voi cong suat hanh trinh
%              400-470 W cua Aerosonde 11-17 kg o 25 m/s (checks/flight_envelope).
%     fc_wkg   cong suat rieng FC + bo dieu khien, W/kg (458): 224 (FLY-150 [R5])
%              hoac 458 (AeroStack 1 kW, 2182 g [R3]).
%     endur_h  thoi gian bay thiet ke de dinh co binh H2, h (1) -- GT.
%     Td       thoi gian dap ung FC dat 95 %, s (10): 1 [R19] .. 10-20 [R24].
%     ramp     gioi han toc do tang dong cua stack 200 W tham chieu, A/s (2) [R25].
%     Np       so nhanh song song pin 13S (1).
%     sc_type  0 khong sieu tu | 1 Yunasko 25 V 7 F 279 g [R5] | 2 Maxwell 16 V 58 F 630 g [R3] (2).
%     esr_sc   ESR sieu tu, Ohm (0.01) -- GT.
%     eta_dc   hieu suat DC-DC (0.95) -- GT.
%     arch     'semi' ban chu dong [R3] | 'passive' [R5][R24] ('semi').
%
%   Mo hinh:
%     FC   duong phan cuc AeroStack 35 cell [R24, Eq. 1], co theo dien tich cell
%          s = P_fc/200; tre kich hoat tau = Td/3 [R19, Eq. 1].
%     H2   binh composite chua 1.4 % H2 theo khoi luong [R26]; luong H2 = 0.6 P_fc *
%          endur / (0.45 * LHV) (GT: tai trung binh 60 % dinh muc, hieu suat FC 45 %).
%     Pin  cell LiFePO4 3.3 V 2.3 Ah Tremblay [R4, Bang 1], 13S (42.9 V danh dinh, gan
%          Vmax 44.4 V cua dong co [R37]); khoi luong 65 Wh/kg (pack 4S 1500 mAh 343 g [R5]) -- GT.
%     Tai nen avionics + nhiem vu 20 W [R25].
if nargin < 1, cfg = struct(); end
def = struct('P_fc', 600, 'fc_wkg', 458, 'endur_h', 1, 'Td', 10, 'ramp', 2, 'Np', 1, ...
    'sc_type', 2, 'esr_sc', 0.010, 'eta_dc', 0.95, 'arch', 'semi');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(cfg, f{k}), cfg.(f{k}) = def.(f{k}); end
end
pp.cfg = cfg;
LHV = 120e6;

%% Pin nhien lieu
s = cfg.P_fc/200;
fc.ref = struct('V0', 30.48, 'B', 0.4947, 'R', 0.5002, 'm', 1.972e-4, 'n', 0.6826, ...
                'Ncell', 35, 'Imax', 10);   % [R24]
fc.scale = s;
if strcmp(cfg.arch, 'passive')
    fc.Ncell = 62;                      % GT: khop bus 13S (~0.7 V/cell khi tai)
else
    fc.Ncell = 35;
end
fc.Imax  = fc.ref.Imax*s;
fc.Imin  = 0.05*fc.Imax;                % GT
fc.P_rated = cfg.P_fc;
fc.tau   = cfg.Td/3;
fc.ramp  = cfg.ramp*s;
fc.eta_F_hi = 0.9; fc.eta_F_lo = 0.4; fc.P_eta_switch = 50*s;   % [R24]
fc.mass  = cfg.P_fc/cfg.fc_wkg;
m_H2 = 0.6*cfg.P_fc*cfg.endur_h*3600/(0.45*LHV);
fc.mass_tank = m_H2/0.014;              % [R26]
fc.deg = struct('load_uV', 0.4185, 'idle_uV_h', 8.662, 'high_uV_h', 10.00, ...
                'startstop_uV', 13.79, 'k', 1.72, 'idle_frac', 0.05, 'high_frac', 0.8);  % [R16][R17]
pp.fc = fc;

%% Pin
bat.cell = struct('E0', 3.366, 'R', 0.01, 'K', 0.0076, 'A', 0.26422, 'B', 26.5487, ...
                  'Q', 2.3, 'Vnom', 3.3);   % [R4, Bang 1]
bat.Ns = 13; bat.Np = cfg.Np;
bat.tau_f = 30;                         % [R4]
bat.Q = bat.cell.Q*bat.Np;
bat.E_Wh = bat.Ns*bat.Np*bat.cell.Vnom*bat.cell.Q;
bat.mass = bat.E_Wh/65;
bat.soc_min = 0.2;
bat.Imax_C = 10;                        % GT; [R18] thu den 10C
bat.age = struct('c', [0.5 2 6 10], 'B', [31630 21681 12934 15512], ...
                 'Ea0', 31700, 'Ea1', 370.3, 'z', 0.55, 'T', 298.15, 'Ah_ref', 1000);  % [R18]
pp.bat = bat;

%% Sieu tu
switch cfg.sc_type
    case 1, sc = struct('use', true, 'C', 7, 'Vmax', 25, 'mass', 0.279, 'Imax', 50);   % [R5]
    case 2, sc = struct('use', true, 'C', 58, 'Vmax', 16, 'mass', 0.630, 'Imax', 100); % [R3]; Imax GT
    otherwise, sc = struct('use', false, 'C', 7, 'Vmax', 25, 'mass', 0, 'Imax', 50);
end
sc.ESR = cfg.esr_sc; sc.e_min = 0.25;   % [R3]
pp.sc = sc;

pp.dc.eta = cfg.eta_dc;
pp.aux.P = 20;
pp.mass_total = fc.mass + fc.mass_tank + bat.mass + sc.mass;
pp.F = 96485; pp.M_H2 = 2.016e-3;
end
