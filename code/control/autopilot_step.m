function [cmd, st, info] = autopilot_step(ap, st, meas, ref, phi_max, dt)
%AUTOPILOT_STEP Mot buoc autopilot (roll/course, pitch/altitude, airspeed/throttle).
%   [cmd, st, info] = AUTOPILOT_STEP(ap, st, meas, ref, phi_max, dt)
%     meas: phi theta p q chi h Va (do truc tiep tu trang thai, gia thiet co du)
%     ref:  chi_c h_c Va_c;  phi_max: bank toi da cua quy dao ung vien
%   Luat [R37, ch. 6]: PI cho course/altitude/airspeed, PD cho roll/pitch.
%   Anti-windup: dung tich phan khi dau ra vong do bi bao hoa (GT).
%   cmd = [der; del; dt] (elevon qua nghich dao tron [R2, Eq. 14]).
if isempty(st), st = struct('i_chi', 0, 'i_h', 0, 'i_V', 0); end

e_chi = wrap_pi(ref.chi_c - meas.chi);
phi_u = ap.kp_chi*e_chi + ap.ki_chi*st.i_chi;
phi_c = min(max(phi_u, -phi_max), phi_max);
if phi_c == phi_u, st.i_chi = st.i_chi + e_chi*dt; end

e_h = ref.h_c - meas.h;
th_u = ap.trim.theta + ap.kp_h*e_h + ap.ki_h*st.i_h;
th_c = min(max(th_u, ap.theta_lim(1)), ap.theta_lim(2));
if th_c == th_u, st.i_h = st.i_h + e_h*dt; end

e_V = ref.Va_c - meas.Va;
dt_u = ap.trim.dt + ap.kp_V*e_V + ap.ki_V*st.i_V;
dtc = min(max(dt_u, 0), 1);
if dtc == dt_u, st.i_V = st.i_V + e_V*dt; end

de = ap.trim.de + ap.kp_th*(th_c - meas.theta) - ap.kd_th*meas.q;
da = ap.trim.da + ap.kp_phi*(phi_c - meas.phi) - ap.kd_phi*meas.p;
[der, del] = x8_elevon_unmix(de, da);
cmd = [der; del; dtc];
info.phi_c = phi_c; info.th_c = th_c; info.sat_phi = phi_c ~= phi_u;
info.sat_th = th_c ~= th_u; info.sat_dt = dtc ~= dt_u;
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
