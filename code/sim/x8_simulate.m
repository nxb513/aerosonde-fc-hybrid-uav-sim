function out = x8_simulate(p, x0, tf, dt, ctrl, opts)
%X8_SIMULATE Mo phong X8 buoc co dinh RK4, co tre lenh va bao hoa servo.
%   out = X8_SIMULATE(p, x0, tf, dt, ctrl, opts)
%     x0    trang thai dau (19 chieu)
%     ctrl  @(t, x, cs) -> [cmd, cs]; cmd = [der_c; del_c; dt_c]
%           hoac vector 3x1 lenh co dinh
%     opts  (tuy chon)
%       .wind  @(t, x) -> [w_ned; g_b] (6x1); mac dinh khong gio
%       .Ub    @(t, x) -> dien ap bus; mac dinh p.bus.U_nom
%       .cs0   trang thai ban dau cua bo dieu khien
%       .cmd0  lenh truoc t = 0 (lap day bo dem tre); mac dinh = lenh tai t = 0
%
%   Tre lenh: elevon p.act.elevon.delay, ga p.act.throttle.delay [R2, Bang 9],
%   lam tron theo buoc dt. Lenh giu nguyen trong mot buoc (ZOH).
%   Co bao hoa (de lam nhan cho AFP):
%     sat_pos  lenh elevon vuot +-max
%     sat_rate toc do elevon cham +-rate

if nargin < 6, opts = struct(); end
I = x8_state_index();
if ~isfield(opts, 'wind'), opts.wind = @(t, x) zeros(6, 1); end
if ~isfield(opts, 'Ub'),   opts.Ub   = @(t, x) p.bus.U_nom; end
if ~isfield(opts, 'cs0'),  opts.cs0  = []; end
if isnumeric(ctrl)
    ucst = ctrl(:);
    ctrl = @(t, x, cs) deal(ucst, cs);
end

N  = round(tf/dt);
nE = round(p.act.elevon.delay/dt);
nT = round(p.act.throttle.delay/dt);

cs = opts.cs0;
[c0, cs] = ctrl(0, x0, cs);
if isfield(opts, 'cmd0'), cprev = opts.cmd0(:); else, cprev = c0(:); end
bufE = repmat(cprev(1:2), 1, nE + 1);
bufT = repmat(cprev(3), 1, nT + 1);

t  = (0:N)'*dt;
X  = zeros(N + 1, I.n);  X(1, :) = x0(:).';
Craw = zeros(N + 1, 3);  Capp = zeros(N + 1, 3);
Va = zeros(N + 1, 1); al = Va; be = Va; Pel = Va; Ibus = Va; Ubv = Va; T = Va;
sat_pos = false(N + 1, 1); sat_rate = sat_pos;

x = x0(:);
emax = p.act.elevon.max; erate = p.act.elevon.rate;
for k = 1:N + 1
    tk = t(k);
    if k > 1
        [c, cs] = ctrl(tk, x, cs);
    else
        c = c0;
    end
    c = c(:);
    bufE = [bufE(:, 2:end), c(1:2)];
    bufT = [bufT(2:end), c(3)];
    ca = [bufE(:, 1); bufT(1)];

    wv = opts.wind(tk, x);
    Ub = opts.Ub(tk, x);
    [~, aux] = x8_derivatives(x, ca, Ub, wv(1:3), wv(4:6), p);

    Craw(k, :) = c.'; Capp(k, :) = ca.';
    Va(k) = aux.Va; al(k) = aux.alpha; be(k) = aux.beta;
    Pel(k) = aux.prop.Pel; Ibus(k) = aux.prop.Ibus; Ubv(k) = Ub; T(k) = aux.prop.T;
    sat_pos(k)  = any(abs(c(1:2)) > emax);
    sat_rate(k) = any(abs(x(I.ddlt)) >= erate*(1 - 1e-6));
    if k == N + 1, break; end

    f = @(xx) x8_derivatives(xx, ca, Ub, wv(1:3), wv(4:6), p);
    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    x(I.q) = x(I.q)/norm(x(I.q));
    x(I.dlt)  = min(max(x(I.dlt), -emax), emax);
    x(I.ddlt) = min(max(x(I.ddlt), -erate), erate);
    x(I.Om)   = max(x(I.Om), 0);
    X(k + 1, :) = x.';
end

out.t = t; out.x = X; out.cmd_raw = Craw; out.cmd_applied = Capp;
out.Va = Va; out.alpha = al; out.beta = be;
out.Pel = Pel; out.Ibus = Ibus; out.Ub = Ubv; out.T = T;
out.sat_pos = sat_pos; out.sat_rate = sat_rate;
out.eul = zeros(N + 1, 3);
for k = 1:N + 1
    out.eul(k, :) = quat_utils('to_euler', X(k, I.q).').';
end
out.E_el = cumtrapz(t, Pel);    % nang luong dien da dung (J)
end
