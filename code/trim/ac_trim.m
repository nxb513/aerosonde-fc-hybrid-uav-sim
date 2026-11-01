function tr = ac_trim(p, Va, gamma, opts)
%AC_TRIM Can bang Aerosonde: bay thang hoac quay vong deu, phoi hop (beta = 0), khong gio.
%   tr = AC_TRIM(p, Va, gamma, opts); opts.psidot (rad/s), opts.Ub (V).
%   Bai toan trim [R37, muc 5.3, App. F]. Co rudder nen ep beta = 0 (quay vong
%   phoi hop). An z = [alpha phi theta de da dr dt]; 7 phuong trinh:
%   dv = 0 (3), dom = 0 (3), van toc thang dung = -Va sin(gamma).
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'psidot'), opts.psidot = 0; end
if ~isfield(opts, 'Ub'), opts.Ub = 40; end
phi0 = atan(Va*opts.psidot/p.env.g);
% Khoi tao ga: nho nhat sao cho luc day >= luc can uoc luong (L/D ~ 10) + thanh phan
% trong luc khi leo. Dong co KV thap nen ga thap thi canh quat quay tu do (J lon),
% fsolve bat dau o do se bi ket o vung phang.
W = p.mass.m*p.env.g; Treq = W/10/cos(phi0) + W*sin(gamma);
dt0 = 1;
for d = 0.3:0.02:1
    if ac_propulsion(Va, d, opts.Ub, p).T >= Treq, dt0 = d; break; end
end
z0 = [0.05; phi0; gamma + 0.05; -0.1; 0; 0; dt0];
fopt = optimoptions('fsolve', 'Display', 'off', 'Algorithm', 'levenberg-marquardt', ...
    'FunctionTolerance', 1e-14, 'StepTolerance', 1e-14, 'MaxIterations', 400, ...
    'MaxFunctionEvaluations', 8000);
[z, res, flag] = fsolve(@(z) resid(z, p, Va, gamma, opts), z0, fopt);
[~, s] = resid(z, p, Va, gamma, opts);

I = ac_state_index();
x0 = zeros(I.n, 1);
x0(I.vel) = s.v; x0(I.q) = s.q; x0(I.om) = s.om;
x0(I.dlt) = z(4:6); x0(I.thr) = z(7);

tr.Va = Va; tr.gamma = gamma; tr.psidot = opts.psidot; tr.Ub = opts.Ub;
tr.alpha = z(1); tr.phi = z(2); tr.theta = z(3);
tr.de = z(4); tr.da = z(5); tr.dr = z(6); tr.dt = z(7);
tr.x0 = x0; tr.cmd = z(4:7);
tr.res_norm = norm(res); tr.exitflag = flag;
tr.converged = flag > 0 && tr.res_norm < 1e-8;
tr.Pel = s.aux.prop.Pel; tr.T = s.aux.prop.T; tr.Om = s.aux.prop.Om;
tr.margin = min(p.act.max - abs(z(4:6)));          % rad, am = vuot gioi han lai
tr.ok_surf = tr.margin >= 0;
tr.ok_thr = z(7) >= 0 && z(7) <= 1;
tr.ok_alpha = abs(z(1)) <= p.alpha_lim;
tr.feasible = tr.converged && tr.ok_surf && tr.ok_thr && tr.ok_alpha;
end

function [r, s] = resid(z, p, Va, gamma, opts)
a = z(1); ph = z(2); th = z(3);
v = Va*[cos(a); 0; sin(a)];
q = quat_utils('from_euler', [ph; th; 0]);
om = opts.psidot*[-sin(th); sin(ph)*cos(th); cos(ph)*cos(th)];
[F, M, aux] = ac_forces_moments(v, q, om, z(4:6), z(7), opts.Ub, zeros(3,1), zeros(3,1), p);
vned = quat_utils('dcm', q)*v;
r = [(F/p.mass.m - cross(om, v))/p.env.g; p.mass.Jinv*(M - cross(om, p.mass.J*om)); vned(3)/Va + sin(gamma)];
s.v = v; s.q = q; s.om = om; s.aux = aux;
end
