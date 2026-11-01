function p = x8_apply_config(p, comps)
%X8_APPLY_CONFIG Cap nhat khoi luong, CG va quan tinh khi lap/thao linh kien.
%   p = X8_APPLY_CONFIG(p, comps) voi comps la mang struct:
%     .name  ten linh kien
%     .m     khoi luong (kg); am = thao bo (vd. thao pin goc)
%     .r     vi tri CG linh kien trong he than, so voi CG danh dinh (m), [x;y;z]
%     .J     quan tinh rieng quanh CG linh kien (3x3, kg m^2); [] = chat diem
%
%   Ket qua:
%     p.mass.m     tong khoi luong
%     p.mass.r_cg  vi tri CG moi so voi goc he than (= CG danh dinh = tam khi dong)
%     p.mass.J     quan tinh quanh CG moi
%
%   Co hoc vat ran (dinh ly truc song song). Goc he than cua [R2] trung tam
%   khi dong va CG danh dinh, nen CG lech khoi goc thi momen phai chuyen ve
%   CG moi [R1, Eq. 18]; viec do lam trong x8_forces_moments.

m  = p.mass.m0;
mr = zeros(3, 1);
JO = p.mass.J0;                 % quan tinh quanh goc O (= CG danh dinh)
for k = 1:numel(comps)
    c = comps(k);
    r = c.r(:);
    Jc = zeros(3);
    if isfield(c, 'J') && ~isempty(c.J)
        Jc = c.J;
    end
    m  = m + c.m;
    mr = mr + c.m*r;
    JO = JO + Jc + c.m*((r.'*r)*eye(3) - r*r.');
end
if m <= 0
    error('x8_apply_config:mass', 'Tong khoi luong phai duong.');
end
r_cg = mr/m;
J = JO - m*((r_cg.'*r_cg)*eye(3) - r_cg*r_cg.');

p.mass.comps = comps;
p.mass.m     = m;
p.mass.r_cg  = r_cg;
p.mass.J     = J;
p.mass.Jinv  = inv(J);
end
