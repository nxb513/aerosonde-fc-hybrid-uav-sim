function pp = power_params(cfg)
%POWER_PARAMS Tham so he nguon lai FC/pin/sieu tu cho X8.
%   pp = POWER_PARAMS(cfg), cfg co the bo trong (mac dinh). Cac truong cfg:
%     P_fc     cong suat dinh muc FC (W): 150 | 200 | 250
%     Td       thoi gian dap ung FC 95% (s): 1 | 10 | 20
%     ramp     gioi han toc do tang dong FC (A/s): 2 | 5
%     Np       so nhanh song song cua pin 5S: 1 | 2
%     use_sc   co sieu tu hay khong
%     esr_sc   ESR sieu tu (Ohm): 0.010 | 0.030
%     eta_dc   hieu suat DC-DC: 0.93 | 0.97
%     arch     'semi' (ban chu dong [R3]) | 'passive' ([R5][R24])
%
%   Nguon so lieu (xem EXPERIMENTS.md muc 1):
%     FC: duong phan cuc AeroStack 35 cell, 200 W [R24, Eq. 1]; tau = Td/3 [R19];
%         khoi luong 224 W/kg (FLY-150 150 W, 0.67 kg [R5]); binh H2 1.09 kg [R5].
%     Pin: cell LiFePO4 3.3 V 2.3 Ah, Tremblay [R4, Bang 1]; khoi luong pack theo
%          65 Wh/kg (pack 4S 1500 mAh 343 g [R5]) -- GT.
%     Sieu tu: Yunasko 25 V 7 F 279 g, xa <= 50 A xung [R5]; e_sc >= 0.25 [R3].
%     Tai nen avionics + nhiem vu 20 W [R25].

if nargin < 1, cfg = struct(); end
def = struct('P_fc', 200, 'Td', 10, 'ramp', 2, 'Np', 1, 'use_sc', true, ...
    'esr_sc', 0.010, 'eta_dc', 0.95, 'arch', 'semi');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(cfg, f{k}), cfg.(f{k}) = def.(f{k}); end
end
pp.cfg = cfg;

%% Pin nhien lieu
s = cfg.P_fc/200;                       % he so co dien tich cell so voi AeroStack
fc.ref = struct('V0', 30.48, 'B', 0.4947, 'R', 0.5002, 'm', 1.972e-4, 'n', 0.6826, ...
                'Ncell', 35, 'Imax', 10);   % [R24]
fc.scale = s;
if strcmp(cfg.arch, 'passive')
    fc.Ncell = 25;                      % 25 cell nhu FLY-150 de khop bus 5S [R5]
else
    fc.Ncell = 35;
end
fc.Imax  = fc.ref.Imax*s;
fc.Imin  = 0.05*fc.Imax;                % GT: dong toi thieu khi chay
fc.P_rated = cfg.P_fc;
fc.tau   = cfg.Td/3;                    % [R19, Eq. 1]
fc.ramp  = cfg.ramp*s;                  % A/s, co theo dien tich
fc.eta_F_hi = 0.9; fc.eta_F_lo = 0.4; fc.P_eta_switch = 50*s;   % [R24]
fc.mass  = cfg.P_fc/224;                % kg (GT tu [R5])
fc.mass_tank = 1.09;                    % kg [R5]
% Suy giam (mo cell) [R17, Bang 1], he so k = 1.72 [R16][R17]
fc.deg = struct('load_uV', 0.4185, 'idle_uV_h', 8.662, 'high_uV_h', 10.00, ...
                'startstop_uV', 13.79, 'k', 1.72, 'idle_frac', 0.05, 'high_frac', 0.8);
pp.fc = fc;

%% Pin (cell LiFePO4, Tremblay [R4, Bang 1])
bat.cell = struct('E0', 3.366, 'R', 0.01, 'K', 0.0076, 'A', 0.26422, 'B', 26.5487, ...
                  'Q', 2.3, 'Vnom', 3.3);
bat.Ns = 5; bat.Np = cfg.Np;
bat.tau_f = 30;                         % s, loc dong i* [R4]
bat.Q = bat.cell.Q*bat.Np;              % Ah cua pack
bat.E_Wh = bat.Ns*bat.Np*bat.cell.Vnom*bat.cell.Q;
bat.mass = bat.E_Wh/65;                 % kg (GT, moc [R5])
bat.soc_min = 0.2;                      % [R4]
bat.Imax_C = 10;                        % C-rate toi da (GT; [R18] thu den 10C)
% Lao hoa [R18, Eq. 7, Bang 3]
bat.age = struct('c', [0.5 2 6 10], 'B', [31630 21681 12934 15512], ...
                 'Ea0', 31700, 'Ea1', 370.3, 'z', 0.55, 'T', 298.15, 'Ah_ref', 1000);
pp.bat = bat;

%% Sieu tu [R5]
sc.use = cfg.use_sc;
sc.C = 7; sc.Vmax = 25; sc.ESR = cfg.esr_sc;
sc.e_min = 0.25;                        % [R3]
sc.Imax = 50;                           % A xung [R5]
sc.mass = 0.279*cfg.use_sc;
pp.sc = sc;

%% DC-DC, tai nen
pp.dc.eta = cfg.eta_dc;
pp.aux.P = 20;                          % W [R25]
pp.mass_total = fc.mass + fc.mass_tank + bat.mass + sc.mass;
pp.F = 96485; pp.M_H2 = 2.016e-3;
end
