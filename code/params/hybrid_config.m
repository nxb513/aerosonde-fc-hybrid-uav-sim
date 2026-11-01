function [comps, info] = hybrid_config(pp, dx_cg, m_remove)
%HYBRID_CONFIG Danh sach linh kien he nguon lai lap len X8.
%   [comps, info] = HYBRID_CONFIG(pp, dx_cg, m_remove)
%     pp        power_params (khoi luong FC, binh H2, pin, sieu tu)
%     dx_cg     dich CG rong theo x mong muon so voi CG danh dinh (m, duong = mui)
%     m_remove  khoi luong pin goc thao ra (kg), tai CG danh dinh (GT, luoi {0, 0.5})
%   Vi tri co so cua linh kien (GT): binh H2 +0.10 m, pin +0.05 m, sieu tu 0,
%   FC -0.05 m, deu thap hon CG 0.02 m. Sau do tinh tien chung theo x de CG rong
%   bang dx_cg. Quan tinh tinh theo chat diem (x8_apply_config).
base = [0.10 0.05 0.00 -0.05];
mass = [pp.fc.mass_tank, pp.bat.mass, pp.sc.mass, pp.fc.mass];
names = {'binh_H2', 'pin', 'sieu_tu', 'FC'};
keep = mass > 0;
base = base(keep); mass = mass(keep); names = names(keep);
m0 = 3.364;
m_tot = m0 - m_remove + sum(mass);
d = (m_tot*dx_cg - sum(mass.*base))/sum(mass);
comps = struct('name', {}, 'm', {}, 'r', {}, 'J', {});
for k = 1:numel(mass)
    comps(end+1) = struct('name', names{k}, 'm', mass(k), 'r', [base(k) + d; 0; 0.02], 'J', []); %#ok<AGROW>
end
if m_remove > 0
    comps(end+1) = struct('name', 'thao_pin_goc', 'm', -m_remove, 'r', [0; 0; 0], 'J', []);
end
info.m_added = sum(mass) - m_remove;
info.shift = d;
end
