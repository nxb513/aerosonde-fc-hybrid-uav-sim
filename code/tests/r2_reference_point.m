function [op, ref] = r2_reference_point(p)
%R2_REFERENCE_POINT Diem hoat dong va ma tran A tham chieu tu [R2, Eq. 36-42, Bang 10-11].
%   [op, ref] = R2_REFERENCE_POINT(p)
%     op   struct dung cho x8_linearize (x0, der, del, dt, Ub) + gio than g_b
%     ref  ma tran A_lon, A_lat va cac cuc do [R2] cong bo
%   Gio [uw vw ww] cua [R2, Eq. 37] la van toc gio trong he than (kiem tra:
%   u - uw = 17.73, w - ww = 2.47 -> alpha = 7.9 deg, Va = 17.9 m/s, khop Eq. 38).
%   Toc do canh quat tai diem nay [R2] khong cong bo: lay Om de dOm = 0.
%   A_lon doc tu Eq. 42; A_lat tu Eq. 40 (ban PDF trich xuat bi tron cot,
%   da sap lai theo hang/cot cua A day du Eq. 44).

d2r = pi/180;
I = x8_state_index();
v   = [11.8; -6.26; -0.28];
eul = [-0.47; -1.11; 92.89]*d2r;
g_b = [-5.93; -6.62; -2.75];
de = -2.35*d2r; da = -2.16*d2r; dt = 0.44;
[der, del] = x8_elevon_unmix(de, da);
Ub = p.bus.U_nom;

vr = v - g_b; Va = norm(vr);
Om = fzero(@(Om) getfield(x8_propulsion(Va, Om, dt, Ub, p), 'dOm'), [50 2000]); %#ok<GFLD>

x0 = zeros(I.n, 1);
x0(I.vel) = v; x0(I.q) = quat_utils('from_euler', eul); x0(I.Om) = Om;
x0(I.dlt) = [der; del]; x0(I.thr) = dt;
op = struct('x0', x0, 'der', der, 'del', del, 'dt', dt, 'Ub', Ub, 'g_b', g_b);

ref.A_lon = [-0.59  0.92  0.61 -9.82;
             -0.23 -6.31  4.42  0.19;
              0.78 -5.59 -5.92  0;
              0     0     1     0];
ref.A_lat = [-0.68  -0.96 -11.52 9.82 0;
             -5.45 -16.81   1.80 0    0;
              0.53  -1.62  -2.10 0    0;
              0      1      0    0    0;
              0      0      1    0    0];
ref.eig_lon = [-6.19+4.93i; -6.19-4.93i; -0.22+0.97i; -0.22-0.97i];
ref.eig_lat = [-17.50; -0.84+3.73i; -0.84-3.73i; -0.42];
end
