function lin = x8_linearize(p, tr, opts)
%X8_LINEARIZE Tuyen tinh hoa X8 quanh mot diem hoat dong (trang thai Euler).
%   lin = X8_LINEARIZE(p, tr, opts)
%     tr    ket qua x8_trim (hoac struct co x0, der, del, dt, Ub)
%     opts.w_ned, opts.g_b  gio giu co dinh khi tuyen tinh hoa (mac dinh 0).
%           g_b co dinh trong he than giong cach [R2, Eq. 37] coi gio la dau vao.
%   Trang thai xe = [u v w p q r phi theta psi Om], dau vao u = [der del dt]
%   (goc elevon/ga thuc te, bo qua servo). Jacobian bang sai phan trung tam
%   [R37, App. F.1]; dong hoc Euler theo [R37, Eq. 3.3].
%   Tach mode nhu [R2, Eq. 39-42]: doc [u w q theta], ngang [v p r phi psi].

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'w_ned'), opts.w_ned = zeros(3, 1); end
if ~isfield(opts, 'g_b'),   opts.g_b   = zeros(3, 1); end
I = x8_state_index();
eul = quat_utils('to_euler', tr.x0(I.q));
xe0 = [tr.x0(I.vel); tr.x0(I.om); eul; tr.x0(I.Om)];
u0  = [tr.der; tr.del; tr.dt];
f = @(xe, u) feul(xe, u, tr.Ub, opts.w_ned, opts.g_b, p);

n = numel(xe0); m = numel(u0);
A = zeros(n); B = zeros(n, m);
for k = 1:n
    h = 1e-6*max(1, abs(xe0(k)));
    e = zeros(n, 1); e(k) = h;
    A(:, k) = (f(xe0 + e, u0) - f(xe0 - e, u0))/(2*h);
end
for k = 1:m
    h = 1e-6*max(1, abs(u0(k)));
    e = zeros(m, 1); e(k) = h;
    B(:, k) = (f(xe0, u0 + e) - f(xe0, u0 - e))/(2*h);
end

ilon = [1 3 5 8];
ilat = [2 4 6 7 9];
lin.A = A; lin.B = B; lin.xe0 = xe0; lin.u0 = u0;
lin.names = {'u','v','w','p','q','r','phi','theta','psi','Om'};
lin.A_lon = A(ilon, ilon); lin.A_lat = A(ilat, ilat);
lin.eig_lon = eig(lin.A_lon);
lin.eig_lat = eig(lin.A_lat);
lin.eig_full = eig(A);
end

function dxe = feul(xe, u, Ub, w_ned, g_b, p)
v = xe(1:3); om = xe(4:6); eul = xe(7:9); Om = xe(10);
q = quat_utils('from_euler', eul);
[F, M, aux] = x8_forces_moments(v, q, om, Om, u(1), u(2), u(3), Ub, w_ned, g_b, p);
dv  = F/p.mass.m - cross(om, v);
dom = p.mass.Jinv*(M - cross(om, p.mass.J*om));
ph = eul(1); th = eul(2);
E = [1 sin(ph)*tan(th) cos(ph)*tan(th);
     0 cos(ph)         -sin(ph);
     0 sin(ph)/cos(th) cos(ph)/cos(th)];
dxe = [dv; dom; E*om; aux.prop.dOm];
end
