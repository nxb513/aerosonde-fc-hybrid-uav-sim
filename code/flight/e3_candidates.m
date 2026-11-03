function C = e3_candidates()
%E3_CANDIDATES Luoi kieu bay cua planner E3 (gioi han lenh gui cho autopilot, khong phai
%   gioi han phan cung).
%   Goc nghieng cho phep {10, 15, ..., 50} deg x goc leo cho phep {2, 4, ..., 10} deg x van
%   toc {22, 23, ..., 28} m/s = 315 kieu bay, trong mien bay tinh bang mo hinh
%   (checks/flight_envelope: bank quay deu toi da 40-57 deg, goc leo toi da 6.5-11 deg,
%   trim 18-31 m/s, stall 16-20 m/s). Kieu bay danh nghia (30 deg, 6 deg, 25 m/s) la diem
%   giua luoi. Bay tham do boc lien tuc trong C.range.
[a, b, c] = ndgrid(10:5:50, 2:2:10, 22:28);
C.traj = [a(:) b(:) c(:)];
C.Va = 22:28;
C.range = [10 50; 2 10; 22 28];
C.nominal_traj = [30 6 25];
C.nominal = find(ismember(C.traj, C.nominal_traj, 'rows'));
end
