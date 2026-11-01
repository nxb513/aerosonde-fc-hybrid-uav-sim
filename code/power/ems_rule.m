function [Pfc, Psc, es] = ems_rule(pp, s, P_dem, dt) %#ok<INUSD>
%EMS_RULE Baseline theo luat: FC bam tai co hieu chinh SoC, khong dung sieu tu.
%   Thuoc lop EMS theo luat/thermostat cua [R27] (FC la nguon chinh, pin bu khi
%   tai vuot kha nang FC va duoc nap khi SoC thap). Luat cu the la GT cua ta.
es = s.ems;
soc = 1 - s.it/pp.bat.cell.Q;
k_soc = 2.0;
Pfc = min(max(P_dem + k_soc*(s.soc0 - soc)*pp.fc.P_rated, 0), pp.fc.P_rated);
Psc = 0;
end
