# Aerosonde fuel-cell hybrid UAV — actuator-aware energy/trajectory simulation (MATLAB)

Mô phỏng UAV Aerosonde [Beard & McLain, App. E] 6-DOF, chuyển đổi sang hệ nguồn lai pin nhiên liệu – pin – siêu tụ, giới hạn cơ cấu chấp hành, gió Dryden và Monte Carlo trên lưới tham số.

- `code/`: mô hình, mô phỏng, lưới thí nghiệm, kiểm thử. Xem `code/README.md` và `code/EXPERIMENTS.md`.
- `.github/workflows/grid.yml`: chạy lưới thành 20 job song song (`gh workflow run grid.yml -f exp=E2`).
- `.github/workflows/tests.yml`: chạy kiểm thử mỗi lần push.

Yêu cầu: MATLAB R2024a + Optimization Toolbox (phần AI cần thêm Statistics and Machine Learning Toolbox).
