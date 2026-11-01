function tr = x8_trim(p, Va, gamma, opts)
%X8_TRIM Diem can bang X8 (bay thang hoac quay vong deu), khong gio.
%   tr = X8_TRIM(p, Va, gamma, opts)
%     Va     van toc khi (m/s);  gamma goc duong bay (rad, duong = leo)
%     opts.psidot  toc do doi huong (rad/s), mac dinh 0 (bay thang)
%     opts.Ub      dien ap bus (V), mac dinh p.bus.U_nom
%
%   Bai toan trim theo [R37, muc 5.3 va App. F]: tim (x*, u*) sao cho
%   dv = 0, dom = 0, dOm = 0 va van toc thang dung = -Va sin(gamma).
%   X8 khong co rudder nen beta, phi, delta_a deu la an. Quay vong: toc do
%   goc than lay tu psidot voi dphi = dtheta = 0 [R37, Eq. 3.2].
%   An: z = [alpha beta phi theta de da dt Om/Om_s], 8 phuong trinh.
%   Khong ep gioi han co cau chap hanh khi giai; kiem tra kha thi sau do.

if nargin < 4, opts = struct(); end
if ~isfield(opts, 'psidot'), opts.psidot = 0; end
if ~isfield(opts, 'Ub'),     opts.Ub = p.bus.U_nom; end
Om_s = 500;

phi0 = atan(Va*opts.psidot/p.env.g);
z0 = [0.12; 0; phi0; gamma + 0.12; -0.04; 0; 0.5; 1.0];
if isfield(opts, 'z0'), z0 = opts.z0; end

fopt = optimoptions('fsolve', 'Display', 'off', 'Algorithm', 'levenberg-marquardt', ...
    'FunctionTolerance', 1e-14, 'StepTolerance', 1e-14, 'MaxIterations', 400, ...
    'MaxFunctionEvaluations', 8000);
[z, res, flag] = fsolve(@(z) resid(z, p, Va, gamma, opts, Om_s), z0, fopt);

[~, s] = resid(z, p, Va, gamma, opts, Om_s);
[der, del] = x8_elevon_unmix(z(5), z(6));

I = x8_state_index();
x0 = zeros(I.n, 1);
x0(I.vel) = s.v;
x0(I.q)   = s.q;
x0(I.om)  = s.om;
x0(I.Om)  = z(8)*Om_s;
x0(I.dlt) = [der; del];
x0(I.thr) = z(7);

e = p.act.elevon;
tr.Va = Va; tr.gamma = gamma; tr.psidot = opts.psidot; tr.Ub = opts.Ub;
tr.alpha = z(1); tr.beta = z(2); tr.phi = z(3); tr.theta = z(4);
tr.de = z(5); tr.da = z(6); tr.der = der; tr.del = del;
tr.dt = z(7); tr.Om = z(8)*Om_s;
tr.x0 = x0; tr.cmd = [der; del; z(7)];
tr.res_norm = norm(res); tr.exitflag = flag;
tr.converged = flag > 0 && tr.res_norm < 1e-8;
tr.Pel = s.aux.prop.Pel; tr.Ibus = s.aux.prop.Ibus; tr.T = s.aux.prop.T;
tr.elevon_margin = e.max - max(abs([der del]));     % rad, am = vuot gioi han
tr.ok_elevon   = tr.elevon_margin >= 0;
tr.ok_throttle = z(7) >= p.act.throttle.min && z(7) <= p.act.throttle.max;
tr.ok_alpha    = z(1) >= p.aero.alpha_valid(1) && z(1) <= p.aero.alpha_valid(2);
tr.feasible    = tr.converged && tr.ok_elevon && tr.ok_throttle && tr.ok_alpha;
end

function [r, s] = resid(z, p, Va, gamma, opts, Om_s)
a = z(1); b = z(2); ph = z(3); th = z(4);
de = z(5); da = z(6); dt = z(7); Om = z(8)*Om_s;
v  = Va*[cos(a)*cos(b); sin(b); sin(a)*cos(b)];
q  = quat_utils('from_euler', [ph; th; 0]);
om = opts.psidot*[-sin(th); sin(ph)*cos(th); cos(ph)*cos(th)];
[der, del] = x8_elevon_unmix(de, da);
[F, M, aux] = x8_forces_moments(v, q, om, Om, der, del, dt, opts.Ub, zeros(3,1), zeros(3,1), p);
dv  = F/p.mass.m - cross(om, v);
dom = p.mass.Jinv*(M - cross(om, p.mass.J*om));
vned = quat_utils('dcm', q)*v;
r = [dv/p.env.g; dom; aux.prop.dOm/Om_s; vned(3)/Va + sin(gamma)];
s.v = v; s.q = q; s.om = om; s.aux = aux;
end
