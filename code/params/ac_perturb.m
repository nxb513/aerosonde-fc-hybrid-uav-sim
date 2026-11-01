function p = ac_perturb(p, seed)
%AC_PERTURB Lay mau bat dinh he so khi dong cho Monte Carlo.
%   p = AC_PERTURB(p, seed); seed <= 0 giu nguyen danh dinh. Moi he so nhan
%   (1 + aero_rel_std*N(0,1)); [R37] khong cho do lech chuan -> GT 10 % (ac_params).
if seed <= 0, return; end
rs = RandStream('mt19937ar', 'Seed', 1e5 + seed);
fn = {'CL0','Cm0','CLa','Cma','CLq','Cmq','CLde','CDde','Cmde','CDp','CYb','Clb','Cnb', ...
      'Clp','Cnp','Clr','Cnr','CYda','Clda','Cnda','CYdr','Cldr','Cndr'};
for k = 1:numel(fn)
    p.aero.(fn{k}) = p.aero.(fn{k})*(1 + p.aero_rel_std*randn(rs));
end
end
