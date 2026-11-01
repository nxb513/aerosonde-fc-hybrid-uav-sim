function [d_dot, dd_dot, th_dot] = x8_actuator_deriv(d, dd, th, cmd, p)
%X8_ACTUATOR_DERIV Dong hoc servo elevon va ga.
%   Elevon: khau bac 2 [R2, Bang 9]
%     dd_dot = wn^2 (sat(d_c) - d) - 2 zeta wn dd
%   co gioi han bien do +-max va toc do +-rate [R7]. Cach hien thuc bao hoa
%   (chan dao ham khi da cham gioi han va con day ra ngoai) la GT cua ta.
%   Ga: khau bac 1, tau [R2, Bang 9], lenh bi kep trong [min, max].

e  = p.act.elevon;
dc = min(max(cmd(1:2), -e.max), e.max);

d_dot  = min(max(dd, -e.rate), e.rate);
dd_dot = e.wn^2*(dc - d) - 2*e.zeta*e.wn*dd;
for k = 1:2
    if (dd(k) >= e.rate && dd_dot(k) > 0) || (dd(k) <= -e.rate && dd_dot(k) < 0)
        dd_dot(k) = 0;
    end
    if (d(k) >= e.max && d_dot(k) > 0) || (d(k) <= -e.max && d_dot(k) < 0)
        d_dot(k) = 0;
    end
end

t = p.act.throttle;
thc = min(max(cmd(3), t.min), t.max);
th_dot = (thc - th)/t.tau;
end
