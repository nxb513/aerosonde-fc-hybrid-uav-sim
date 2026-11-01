function E = flight_envelope(masses, Ub, outfile)
%FLIGHT_ENVELOPE Dac tinh bay Aerosonde theo khoi luong -> co so chon luoi tham so.
%   E = FLIGHT_ENVELOPE(masses, Ub, outfile)
%   Voi moi khoi luong (them/bot chat diem tai CG danh dinh) tinh:
%     CL_max cua mo hinh, van toc stall, dai van toc trim kha thi (ga <= 1,
%     alpha <= alpha_lim, lai trong gioi han), cong suat dien bay ngang, van toc
%     cong suat nho nhat, goc leo toi da, bank toi da khi quay vong deu.
%   Ngoai ra: do du tru on dinh doc, gioi han CG, mode tuyen tinh, luc day.
if nargin < 1, masses = [9 11 13 15 17]; end
if nargin < 2, Ub = 40; end
p0 = ac_params(); d2r = pi/180;
fid = 1;
if nargin >= 3, fid = fopen(outfile, 'w'); end

% CL_max cua mo hinh (q = 0, de = 0)
al = (0:0.1:30)*d2r; CL = zeros(size(al));
for k = 1:numel(al)
    o = ac_aero(20*[cos(al(k)); 0; sin(al(k))], [0;0;0], [0;0;0], p0); CL(k) = o.CL;
end
[CLmax, k] = max(CL);
p47 = p0; p47.aero.alpha0 = p0.aero.alpha0_R37; CL47 = 0*al;
for j = 1:numel(al)
    o = ac_aero(20*[cos(al(j)); 0; sin(al(j))], [0;0;0], [0;0;0], p47); CL47(j) = o.CL;
end
fprintf(fid, '# Dac tinh bay Aerosonde (U_bus = %.0f V)\n\n', Ub);
fprintf(fid, '- CL_max mo hinh: %.2f tai alpha = %.1f deg (alpha0 = %.0f deg). Voi alpha0 cua [R37] (27 deg): CL_max = %.2f.\n', ...
    CLmax, al(k)/d2r, p0.aero.alpha0/d2r, max(CL47));
SM = -p0.aero.Cma/p0.aero.CLa;
fprintf(fid, '- Du tru on dinh doc SM = -Cma/CLa = %.3f c (%.1f cm). CG lui 5 cm -> SM = %.2f; giu SM >= 0.10 can dx_cg >= %.1f cm.\n', ...
    SM, SM*p0.geom.c*100, SM - 0.05/p0.geom.c, -(SM - 0.10)*p0.geom.c*100);

% Luc day
o0 = ac_propulsion(1, 1, Ub, p0); o25 = ac_propulsion(25, 1, Ub, p0);
fprintf(fid, '- Luc day ga 1: tinh %.1f N (P = %.0f W), o 25 m/s %.1f N (P = %.0f W).\n\n', o0.T, o0.Pel, o25.T, o25.Pel);

fprintf(fid, '| m (kg) | V_stall | V trim kha thi | V_Pmin | P(V_Pmin) | P(25) | gamma_max(25) | bank_max(25) | alpha(25) | de(25) |\n|---|---|---|---|---|---|---|---|---|---|\n');
Vs_grid = 12:1:40;
E = struct();
for i = 1:numel(masses)
    m = masses(i);
    p = ac_apply_config(p0, struct('name', 'm', 'm', m - p0.mass.m0, 'r', [0;0;0], 'J', []));
    Vst = sqrt(2*m*p.env.g/(p.env.rho*p.geom.S*CLmax));
    P = nan(size(Vs_grid)); feas = false(size(Vs_grid));
    for j = 1:numel(Vs_grid)
        tr = ac_trim(p, Vs_grid(j), 0, struct('Ub', Ub));
        if tr.converged, P(j) = tr.Pel; feas(j) = tr.feasible; end
    end
    Vf = Vs_grid(feas);
    [Pmin, jm] = min(P + 1e9*~feas);
    tr25 = ac_trim(p, 25, 0, struct('Ub', Ub));
    gmax = bisect(@(g) ac_trim(p, 25, g, struct('Ub', Ub)).feasible, 0, 30*d2r);
    wmax = bisect(@(w) ac_trim(p, 25, 0, struct('Ub', Ub, 'psidot', w)).feasible, 0, 1.0);
    bmax = atan(25*wmax/p.env.g);
    fprintf(fid, '| %.0f | %.1f | %s | %d | %.0f W | %.0f W | %.1f deg | %.0f deg | %.1f deg | %.1f deg |\n', ...
        m, Vst, rng_str(Vf), Vs_grid(jm), Pmin, tr25.Pel, gmax/d2r, bmax/d2r, tr25.alpha/d2r, tr25.de/d2r);
    E(i).m = m; E(i).Vstall = Vst; E(i).Vfeas = Vf; E(i).V_Pmin = Vs_grid(jm); E(i).Pmin = Pmin;
    E(i).P25 = tr25.Pel; E(i).gmax25 = gmax; E(i).bmax25 = bmax; E(i).P = P; E(i).V = Vs_grid;
end

% Gioi han CG phia mui: elevator du trim o van toc thap
fprintf(fid, '\n## Anh huong CG (m = 13 kg)\n\n| dx_cg (cm) | SM | de trim 20 m/s | de trim 30 m/s | bien lai (20 m/s) |\n|---|---|---|---|---|\n');
for dx = [-0.08 -0.06 -0.04 -0.02 0 0.02 0.04 0.06 0.08]
    p = ac_apply_config(p0, struct('name', 'm', 'm', 2, 'r', [0;0;0], 'J', []));
    p.mass.r_cg = [dx; 0; 0];
    t1 = ac_trim(p, 20, 0, struct('Ub', Ub)); t2 = ac_trim(p, 30, 0, struct('Ub', Ub));
    fprintf(fid, '| %+.0f | %.2f | %.1f deg | %.1f deg | %.1f deg |\n', dx*100, SM + dx/p0.geom.c, ...
        t1.de/d2r, t2.de/d2r, t1.margin/d2r);
end

% Mode tuyen tinh tai 11 kg, 25 m/s
lin = ac_linearize(p0, ac_trim(p0, 25, 0, struct('Ub', Ub)));
fprintf(fid, '\n## Mode tuyen tinh (11 kg, 25 m/s)\n\n- doc: %s\n- ngang: %s\n', ...
    mat2str(round(lin.eig_lon.', 3)), mat2str(round(lin.eig_lat.', 3)));
if fid ~= 1, fclose(fid); end
end

function x = bisect(ok, lo, hi)
if ~ok(lo), x = NaN; return; end
if ok(hi), x = hi; return; end
for k = 1:25
    mid = (lo + hi)/2;
    if ok(mid), lo = mid; else, hi = mid; end
end
x = lo;
end

function s = rng_str(v)
if isempty(v), s = 'khong co'; else, s = sprintf('%d-%d m/s', min(v), max(v)); end
end
