function C = e3_candidates()
%E3_CANDIDATES Tap ung vien cua planner E3.
%   Quy dao: bank toi da {15, 25, 35, 45} deg x goc leo toi da {2, 5, 8} deg x van toc
%   {22, 25, 28} m/s (checks/flight_envelope). Muc bu nang luong = tan so cat bo loc EMS
%   {0.06, 0.3, 1} Hz (dai thap/cao cua [R3][R15]); f_c thap -> pin/sieu tu bu nhieu hon.
[a, b, c] = ndgrid([15 25 35 45], [2 5 8], [22 25 28]);
C.traj = [a(:) b(:) c(:)];
C.fc = [0.06 0.3 1];
C.Va = [22 25 28];
C.nominal = find(ismember(C.traj, [35 5 25], 'rows'));
end
