function [sigma, L] = dryden_sigma(W_h, h, intensity)
%DRYDEN_SIGMA Cuong do va do dai thang Dryden tang thap (MIL-F-8785C [R12]).
%   [sigma, L] = DRYDEN_SIGMA(W_h, h, intensity), sigma = [su sv sw] (m/s), L = [Lu Lv Lw] (m).
%   Gio tai 20 ft suy nguoc tu profil log u(h) = u20 ln(h/z0)/ln(20/z0), z0 = 2 ft
%   [R12, 3.7.3.2]. sigma_w = 0.1 u20, sigma_u/sigma_w = sigma_v/sigma_w
%   = 1/(0.177 + 0.000823 h)^0.4; L_u = L_v = h/(0.177 + 0.000823 h)^1.2, L_w = h
%   (h tinh bang ft) [R12, 3.7.3]. intensity: he so nhan cuong do (GT, luoi).
ft = 0.3048;
hf = max(h/ft, 10);
z0 = 2;
u20 = W_h/(log(hf/z0)/log(20/z0));
sw = 0.1*u20*intensity;
su = sw/(0.177 + 0.000823*hf)^0.4;
Lu = hf/(0.177 + 0.000823*hf)^1.2*ft;
Lw = hf*ft;
sigma = [su su sw]; L = [Lu Lu Lw];
end
