function r = ds1_simulate_maneuver(p, d, dt)
%DS1_SIMULATE_MANEUVER Mo phong ho vong X8 voi lenh, gio va dien ap do trong [DS1].
%   r = DS1_SIMULATE_MANEUVER(p, d, dt)
%     d   ket qua ds1_load;  dt buoc mo phong (s)
%   Cach lam giong kiem chung "nonlinear model prediction" cua [R2, muc 4.1.2]:
%   - trang thai dau = uoc luong EKF tai mau dau tien;
%   - lenh elevator/aileron do -> elevon qua nghich dao [R2, Eq. 14]; ga do;
%     tre va dong hoc servo theo [R2, Bang 9];
%   - gio NED uoc luong (bien thien theo thoi gian) va dien ap pin do.
%   Omega_p ban dau lay o trang thai can bang momen (khong do).
%   Ngoai ra tinh dong pin "gan tinh" tu ga, Va, U do de tach rieng sai so he day.

I = x8_state_index();
t0 = d.t(1);
tt = d.t - t0;
tf = ceil(tt(end)/dt)*dt;
[der_m, del_m] = x8_elevon_unmix(d.elevator, d.aileron);

cmdfun  = @(t) [interp1(tt, der_m, t, 'previous', 'extrap');
                interp1(tt, del_m, t, 'previous', 'extrap');
                interp1(tt, d.throttle, t, 'previous', 'extrap')];
ctrl    = @(t, x, cs) deal(cmdfun(t), cs);
windfun = @(t, x) [interp1(tt, [d.wn d.we d.wd], t, 'linear', 'extrap').'; zeros(3, 1)];
Ubfun   = @(t, x) interp1(tt, d.Voltage, t, 'linear', 'extrap');

x0 = zeros(I.n, 1);
x0(I.vel) = [d.u(1); d.v(1); d.w(1)];
x0(I.q)   = quat_utils('from_euler', [d.phi(1); d.theta(1); d.psi(1)]);
x0(I.om)  = [d.p(1); d.q(1); d.r(1)];
x0(I.Om)  = steady_Om(d.Va(1), d.throttle(1), d.Voltage(1), p);
x0(I.dlt) = [der_m(1); del_m(1)];
x0(I.thr) = d.throttle(1);

opts = struct('wind', windfun, 'Ub', Ubfun, 'cmd0', cmdfun(0));
s = x8_simulate(p, x0, tf, dt, ctrl, opts);

% Lay mau mo phong tai thoi diem do
ts = s.t;
r.t = tt;
r.sim.alpha = interp1(ts, s.alpha, tt);
r.sim.beta  = interp1(ts, s.beta, tt);
r.sim.Va    = interp1(ts, s.Va, tt);
r.sim.phi   = interp1(ts, s.eul(:, 1), tt);
r.sim.theta = interp1(ts, s.eul(:, 2), tt);
r.sim.p     = interp1(ts, s.x(:, I.om(1)), tt);
r.sim.q     = interp1(ts, s.x(:, I.om(2)), tt);
r.sim.r     = interp1(ts, s.x(:, I.om(3)), tt);
r.sim.Ibus  = interp1(ts, s.Ibus, tt);
r.meas = struct('alpha', d.alpha, 'beta', d.beta, 'Va', d.Va, 'phi', d.phi, ...
    'theta', d.theta, 'p', d.p, 'q', d.q, 'r', d.r, 'Ibus', d.Current);

% Dong pin gan tinh tu ga/Va/U do (tach sai so he day khoi sai so dong hoc bay)
Iq = zeros(size(tt));
for k = 1:numel(tt)
    Om = steady_Om(d.Va(k), d.throttle(k), d.Voltage(k), p);
    o = x8_propulsion(d.Va(k), Om, d.throttle(k), d.Voltage(k), p);
    Iq(k) = o.Ibus;
end
r.Ibus_quasi = Iq;
r.name = d.name;
end

function Om = steady_Om(Va, dt, Ub, p)
f = @(Om) getfield(x8_propulsion(Va, Om, dt, Ub, p), 'dOm'); %#ok<GFLD>
Om = fzero(f, [1 3000]);
end
