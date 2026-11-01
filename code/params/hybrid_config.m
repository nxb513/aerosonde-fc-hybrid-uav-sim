function [comps, info] = hybrid_config(pp, dx_cg, m_remove, m0)
%HYBRID_CONFIG Linh kien he nguon lai lap vao Aerosonde (thay dong co xang + nhien lieu).
%   [comps, info] = HYBRID_CONFIG(pp, dx_cg, m_remove, m0)
%     dx_cg     vi tri CG rong mong muon so voi CG tham chieu cua [R37] (m, duong = mui)
%     m_remove  khoi luong dong co + nhien lieu thao ra (kg), dat tai CG tham chieu (GT)
%     m0        khoi luong danh dinh [R37] (11 kg)
%   Vi tri co so (GT): binh H2 +0.15 m, pin +0.05 m, sieu tu 0, FC -0.10 m, thap
%   hon CG 0.03 m; sau do tinh tien chung theo x de CG rong = dx_cg.
base = [0.15 0.05 0.00 -0.10];
mass = [pp.fc.mass_tank, pp.bat.mass, pp.sc.mass, pp.fc.mass];
names = {'binh_H2', 'pin', 'sieu_tu', 'FC'};
keep = mass > 0;
base = base(keep); mass = mass(keep); names = names(keep);
m_tot = m0 - m_remove + sum(mass);
d = (m_tot*dx_cg - sum(mass.*base))/sum(mass);
comps = struct('name', {}, 'm', {}, 'r', {}, 'J', {});
for k = 1:numel(mass)
    comps(end+1) = struct('name', names{k}, 'm', mass(k), 'r', [base(k) + d; 0; 0.03], 'J', []); %#ok<AGROW>
end
comps(end+1) = struct('name', 'thao_dong_co_xang', 'm', -m_remove, 'r', [0; 0; 0], 'J', []);
info.m_total = m_tot; info.m_added = sum(mass) - m_remove; info.shift = d;
end
