# Aerosonde fuel-cell hybrid UAV — actuator-aware energy/trajectory simulation (MATLAB)

Mô phỏng UAV Aerosonde [Beard & McLain, App. E] 6-DOF, chuyển đổi sang hệ nguồn lai pin nhiên liệu – pin – siêu tụ, giới hạn cứng ở mọi bộ phận, gió Dryden. Máy bay bay chuỗi nhiệm vụ đến khi cạn nhiên liệu; mỗi 2 s, planner AI chọn kiểu bay (góc nghiêng cho phép, góc leo cho phép, vận tốc).

- `code/`: mô hình, mô phỏng, lưới thí nghiệm, kiểm thử. Xem `code/README.md` và `code/EXPERIMENTS.md`.
- `.github/workflows/e3.yml`: thí nghiệm E3, tối đa 20 job song song (thăm dò, huấn luyện, đánh giá; xem `code/EXPERIMENTS.md` mục 7).
- `.github/workflows/tests.yml`: chạy kiểm thử mỗi lần push.

Yêu cầu: MATLAB R2024a + Optimization Toolbox (phần AI cần thêm Statistics and Machine Learning Toolbox).
