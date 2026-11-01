# Aerosonde fuel-cell hybrid UAV — actuator-aware energy/trajectory simulation (MATLAB)

Mô phỏng UAV Aerosonde [Beard & McLain, App. E] 6-DOF, chuyển đổi sang hệ nguồn lai pin nhiên liệu – pin – siêu tụ, giới hạn cơ cấu chấp hành, gió Dryden. Máy bay bay chuỗi nhiệm vụ đến khi cạn nhiên liệu; planner AI quyết định lại quỹ đạo và mức bù năng lượng mỗi 3 s.

- `code/`: mô hình, mô phỏng, lưới thí nghiệm, kiểm thử. Xem `code/README.md` và `code/EXPERIMENTS.md`.
- `.github/workflows/e3.yml`: thí nghiệm E3, tối đa 20 job song song (`gh workflow run e3.yml -f stage=explore`, rồi `-f stage=eval -f explore_run=<run id>`).
- `.github/workflows/tests.yml`: chạy kiểm thử mỗi lần push.

Yêu cầu: MATLAB R2024a + Optimization Toolbox (phần AI cần thêm Statistics and Machine Learning Toolbox).
