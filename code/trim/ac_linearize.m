function lin = ac_linearize(p, tr)
%AC_LINEARIZE Tuyen tinh hoa quanh trim, trang thai Euler [u v w p q r phi theta psi],
%   dau vao [de da dr dt] (goc lai/ga thuc te). Jacobian sai phan trung tam
%   [R37, App. F.1]; dong hoc Euler [R37, Eq. 3.3]. Mode: doc [u w q theta],
%   ngang [v p r phi psi] [R37, muc 5.5].
I = ac_state_index();
eul = quat_utils('to_euler', tr.x0(I.q));
xe0 = [tr.x0(I.vel); tr.x0(I.om); eul];
u0 = tr.cmd(:);
f = @(xe, u) feul(xe, u, tr.Ub, p);
n = 9; m = 4; A = zeros(n); B = zeros(n, m);
for k = 1:n
    h = 1e-6*max(1, abs(xe0(k))); e = zeros(n, 1); e(k) = h;
    A(:, k) = (f(xe0 + e, u0) - f(xe0 - e, u0))/(2*h);
end
for k = 1:m
    h = 1e-6*max(1, abs(u0(k))); e = zeros(m, 1); e(k) = h;
    B(:, k) = (f(xe0, u0 + e) - f(xe0, u0 - e))/(2*h);
end
lin.A = A; lin.B = B; lin.xe0 = xe0; lin.u0 = u0;
lin.A_lon = A([1 3 5 8], [1 3 5 8]); lin.A_lat = A([2 4 6 7 9], [2 4 6 7 9]);
lin.eig_lon = eig(lin.A_lon); lin.eig_lat = eig(lin.A_lat);
end

function dxe = feul(xe, u, Ub, p)
v = xe(1:3); om = xe(4:6); eul = xe(7:9);
q = quat_utils('from_euler', eul);
[F, M] = ac_forces_moments(v, q, om, u(1:3), u(4), Ub, zeros(3,1), zeros(3,1), p);
ph = eul(1); th = eul(2);
E = [1 sin(ph)*tan(th) cos(ph)*tan(th); 0 cos(ph) -sin(ph); 0 sin(ph)/cos(th) cos(ph)/cos(th)];
dxe = [F/p.mass.m - cross(om, v); p.mass.Jinv*(M - cross(om, p.mass.J*om)); E*om];
end
