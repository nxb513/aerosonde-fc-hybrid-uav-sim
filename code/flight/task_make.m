function tk = task_make(sp, p0, chi0, h0)
%TASK_MAKE Hinh hoc cua mot nhiem vu tu dac ta sp (task_specs) va tu the luc bat dau.
%   tk = TASK_MAKE(sp, p0, chi0, h0); p0 = [north; east] (m), chi0 huong bay (rad), h0 (m).
%   tk.legs: mang chang bay, kieu 'line' (a -> b, chuyen chang bang nua mat phang tai b
%   [R37, ch. 11]) hoac 'orbit' (tam c, ban kinh rho, chieu dir = +1 cung chieu kim
%   dong ho, tong goc ang [R37, ch. 10]).
%   Kieu bay danh nghia (30 deg, 6 deg, 25 m/s): cung quay ban kinh R_nom = V^2/(g tan 30 deg),
%   leo/ha o goc 6 deg. tk.L = do dai duong bay danh nghia (m); T_dn = L/25 m/s.
%   Nhiem vu khong co han hoan thanh. Gioi han chong ket: qua T_abort =
%   max(3 T_dn, T_dn + 120 s) thi bo nhiem vu (khong dat).
V = 25; g = 9.81; gam = 6; Rn = V^2/(g*tand(30));
hmin = 60; hmax = 250;
u = sp.u;
e = [cos(chi0); sin(chi0)];
n = [-sin(chi0); cos(chi0)];               % vuong goc ben phai huong bay
dh = 0; turb_mult = 1;
switch sp.type
    case 1  % chuyen chang toi waypoint 1-2.5 km, lech huong +-120 deg
        D = 1000 + 1500*u(1); b = (-120 + 240*u(2))*pi/180;
        legs = leg_line(p0, p0 + D*[cos(chi0 + b); sin(chi0 + b)]);
        Lp = D + Rn*abs(b); name = 'transit';
    case 2  % doi huong 90/135/180 deg roi bay thang 400 m
        mags = [90 135 180];
        dchi = mags(1 + floor(3*u(1)))*sign(u(2) - 0.5)*pi/180;
        legs = leg_line(p0, p0 + 400*[cos(chi0 + dchi); sin(chi0 + dchi)]);
        Lp = 400 + Rn*abs(dchi); name = 'turn';
    case 3  % leo/ha 20-60 m tren huong hien tai
        dh = (20 + 40*u(1))*sign(u(2) - 0.5);
        if h0 + dh > hmax || h0 + dh < hmin, dh = -dh; end
        legs = leg_line(p0, p0 + 1e5*e);
        Lp = abs(dh)/sind(gam); name = 'climb';
    case 4  % bay vong cho 2 vong, ban kinh 120-250 m, vao tiep tuyen
        rho = 120 + 130*u(1); dirn = sign(u(2) - 0.5);
        legs = leg_orbit(p0 + dirn*rho*n, rho, dirn, 4*pi);
        Lp = 4*pi*rho; name = 'loiter';
    case 5  % quet luong cay 4 duong, dai 400-700 m, cach 80-160 m
        L = 400 + 300*u(1); s = 80 + 80*u(2); side = sign(u(3) - 0.5); nl = 4;
        a = p0; legs = leg_line(a, a + L*e);
        for i = 2:nl
            a = legs(end).b + side*s*n;
            legs(end+1) = leg_line(a, a + L*e*(-1)^(i - 1)); %#ok<AGROW>
        end
        Lp = nl*L + (nl - 1)*(pi*Rn + max(0, 2*Rn - s)); name = 'survey';
    otherwise  % giu quy dao thang 1.5 km trong gio giat (cuong do nhieu x2, GT)
        legs = leg_line(p0, p0 + 1500*e);
        Lp = 1500; turb_mult = 2; name = 'gust_hold';
end
tk.type = sp.type; tk.name = name; tk.legs = legs; tk.seg = 1;
tk.h_t = h0 + dh; tk.gam_nom = gam*pi/180;
tk.L = Lp; tk.T_dn = Lp/V;
tk.T_abort = max(3*tk.T_dn, tk.T_dn + 120);
tk.turb_mult = turb_mult;
tk.ang_done = 0; tk.ang_prev = NaN;
tk.len_after = zeros(1, numel(legs));      % do dai cac chang con lai sau chang i
for i = 1:numel(legs)
    tk.len_after(i) = sum(arrayfun(@leg_len, legs(i+1:end)));
end
end

function l = leg_line(a, b)
l = struct('kind', 'line', 'a', a, 'b', b, 'c', [0; 0], 'rho', 0, 'dir', 0, 'ang', 0);
end

function l = leg_orbit(c, rho, dirn, ang)
l = struct('kind', 'orbit', 'a', [0; 0], 'b', [0; 0], 'c', c, 'rho', rho, 'dir', dirn, 'ang', ang);
end

function L = leg_len(l)
if strcmp(l.kind, 'line'), L = norm(l.b - l.a); else, L = l.ang*l.rho; end
end
