function R = power_ablation(T2, traces, rows, dt)
%POWER_ABLATION Ablation phia nguon tren cac chuoi tai da chon (posthoc_power).
%   Bien the: EMS loc tan so [R3] (tham chieu) | theo luat [R27] | ECMS [R30]
%   | A5 khong sieu tu | A9 kien truc thu dong [R5] | S1 Td = 1 / 20 s | pin 5S2P.
if nargin < 4, dt = 0.05; end
V = {
  'filter_ref',  struct(),                          @ems_filter
  'rule',        struct(),                          @ems_rule
  'ecms',        struct(),                          @ems_ecms
  'A5_no_sc',    struct('use_sc', false),           @ems_filter
  'A9_passive',  struct('arch', 'passive'),         @ems_filter
  'S1_Td1',      struct('Td', 1),                   @ems_filter
  'S1_Td20',     struct('Td', 20),                  @ems_filter
  'bat_5S2P',    struct('Np', 2),                   @ems_filter
};
R = table();
for v = 1:size(V, 1)
    m = zeros(numel(rows), 6);
    for i = 1:numel(rows)
        k = rows(i);
        cfg = V{v, 2}; cfg.P_fc = T2.in_P_fc(k);
        r = posthoc_power(traces{k}.P, dt, cfg, V{v, 3});
        m(i, :) = [r.m_H2_eq*1e3, r.fc_uV, r.bat_loss_pct*1e3, r.bat_limit_t, r.V_min, r.I_bat_max];
    end
    R = [R; table(string(V{v, 1}), mean(m(:, 1)), mean(m(:, 2)), mean(m(:, 3)), ...
        mean(m(:, 4) > 0), mean(m(:, 5)), mean(m(:, 6)), ...
        'VariableNames', {'variant','H2_eq_g','fc_dV_uV','bat_loss_mpct', ...
        'frac_bat_limit','V_bus_min','I_bat_max'})]; %#ok<AGROW>
end
end
