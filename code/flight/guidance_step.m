function [g, tk] = guidance_step(tk, p, chi, h, Vg, psi)
%GUIDANCE_STEP Bam duong theo truong vector [R37, ch. 10] cho nhiem vu tk.
%   [g, tk] = GUIDANCE_STEP(tk, p, chi, h, Vg, psi); p = [north; east].
%   Duong thang: chi_c = chi_q - chi_inf (2/pi) atan(k_path e_py) [R37, Eq. 10.9];
%     chuyen chang khi qua nua mat phang tai diem cuoi [R37, ch. 11].
%   Quy dao tron: chi_c = phi_ang + dir [pi/2 + atan(k_orbit (d - rho)/rho)] [R37, Eq. 10.15],
%     roll feed-forward phi_ff = dir atan(Vg^2/(rho g cos(chi - psi))) [R37, Eq. 10.18].
%   Tham so truong vector (GT): chi_inf = 60 deg, k_path = 0.02 1/m, k_orbit = 2.
%   Dau ra: chi_c, phi_ff, e_py (sai lech ngang), r (quang duong con lai tuong duong, m:
%   max(duong con lai, |h_t - h|/sin 5 deg)), kappa (do cong chang hien tai), d_wp (khoang
%   cach toi diem chuyen chang), turn_next (goc doi huong o chang ke), done.
chi_inf = pi/3; k_path = 0.02; k_orbit = 2; grav = 9.81;
nl = numel(tk.legs);
g = struct('chi_c', chi, 'phi_ff', 0, 'e_py', 0, 'r', 0, 'kappa', 0, 'd_wp', 0, ...
           'turn_next', 0, 'done', false);
while true
    l = tk.legs(tk.seg);
    if strcmp(l.kind, 'line')
        q = (l.b - l.a)/norm(l.b - l.a);
        chi_q = atan2(q(2), q(1));
        e_py = -sin(chi_q)*(p(1) - l.a(1)) + cos(chi_q)*(p(2) - l.a(2));
        s_rem = (l.b - p).'*q;
        if s_rem <= 0 && tk.seg < nl
            tk.seg = tk.seg + 1; continue
        end
        g.chi_c = chi_q - chi_inf*2/pi*atan(k_path*e_py);
        g.e_py = e_py; g.d_wp = max(s_rem, 0);
        if tk.seg < nl
            q2 = tk.legs(tk.seg + 1).b - tk.legs(tk.seg + 1).a;
            g.turn_next = abs(mod(atan2(q2(2), q2(1)) - chi_q + pi, 2*pi) - pi);
        end
        path_rem = max(s_rem, 0) + tk.len_after(tk.seg);
        if tk.type == 3, path_rem = 0; end           % leo/ha: chi tinh theo do cao
        g.done = tk.seg == nl && s_rem <= 0 && tk.type ~= 3;
    else
        dvec = p - l.c; d = norm(dvec);
        ang = atan2(dvec(2), dvec(1));
        if ~isnan(tk.ang_prev)
            tk.ang_done = tk.ang_done + l.dir*(mod(ang - tk.ang_prev + pi, 2*pi) - pi);
        end
        tk.ang_prev = ang;
        g.chi_c = ang + l.dir*(pi/2 + atan(k_orbit*(d - l.rho)/l.rho));
        g.phi_ff = l.dir*atan(Vg^2/(l.rho*grav*max(cos(chi - psi), 0.5)));
        g.e_py = d - l.rho; g.kappa = 1/l.rho;
        rem = max(l.ang - tk.ang_done, 0)*l.rho;
        g.d_wp = rem;
        path_rem = rem + tk.len_after(tk.seg);
        g.done = tk.ang_done >= l.ang && tk.seg == nl;
        if tk.ang_done >= l.ang && tk.seg < nl
            tk.seg = tk.seg + 1; tk.ang_prev = NaN; continue
        end
    end
    break
end
g.r = max(path_rem, abs(tk.h_t - h)/sind(5));
if tk.type == 3, g.done = abs(tk.h_t - h) < 3; end
end
