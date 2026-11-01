# X8 fuel-cell hybrid UAV — actuator-aware energy/trajectory simulation (MATLAB)

Mô phỏng UAV cánh bay Skywalker X8 6-DOF có hệ nguồn lai pin nhiên liệu – pin – siêu tụ, giới hạn cơ cấu chấp hành, gió Dryden và Monte Carlo trên lưới tham số.

- `code/`: mô hình, mô phỏng, lưới thí nghiệm, kiểm thử. Xem `code/README.md` và `code/EXPERIMENTS.md`.
- `data/DS1/`: dữ liệu bay X8 của Løw-Hansen & Hann, DataverseNO (2024), doi:10.18710/U4TLYV, giấy phép CC0 1.0.
- `.github/workflows/grid.yml`: chạy lưới thành 20 job song song (`gh workflow run grid.yml -f exp=E2`).
- `.github/workflows/tests.yml`: chạy kiểm thử mỗi lần push.

Yêu cầu: MATLAB R2024a + Optimization Toolbox (phần AI cần thêm Statistics and Machine Learning Toolbox).
