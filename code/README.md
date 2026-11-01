# Aerosonde FC-hybrid UAV — mô phỏng 6-DOF có giới hạn cơ cấu chấp hành (MATLAB)

Nền máy bay là **Aerosonde** [R37, App. E], chuyển đổi từ động cơ xăng sang hệ nguồn lai **pin nhiên liệu – pin – siêu tụ**. Bản X8 trước đây đã bị hủy vì X8 (3.4 kg) không chở nổi hệ FC. Thiết kế thí nghiệm, nguồn tham số và các giả thiết (GT) nằm trong `EXPERIMENTS.md`.

## Chạy

```matlab
cd code
run_tests                                   % 15 test
setup_paths; flight_envelope([11 13 15 17], 42.9, 'results/flight_envelope.md')
```

Chạy lưới trên GitHub: `gh workflow run grid.yml -f exp=E2` (hoặc `E1`). Phân tích: `ai/run_analysis(dirE1, dirE2)`.

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `params/` | `ac_params` (Aerosonde [R37], servo, α0); `ac_apply_config` (m, CG, J); `hybrid_config` (bố trí FC/bình/pin/siêu tụ) |
| `model/` | Khí động có stall và cản bậc hai [R37, Eq. 4.6–4.26]; động cơ + cánh quạt nối bus [R37, Eq. 4.17–4.21]; 6-DOF quaternion; servo bão hòa |
| `trim/` | Trim phối hợp (β = 0), tuyến tính hóa |
| `control/` | Autopilot [R37, ch. 6] + yaw damper; vòng pitch bền theo CG/trễ |
| `power/` | FC [R24][R19], pin [R4][R18], siêu tụ, DC-DC, EMS (lọc tần số [R3], luật [R27], ECMS [R30]), kiến trúc thụ động |
| `wind/` | Dryden tầng thấp [R12] |
| `mission/` | Đoạn nhiệm vụ, nhãn bão hòa từng mặt lái |
| `grid/` | Lưới E1/E2, chia chunk, gộp kết quả |
| `ai/` | AFP (rừng lượng tử + CQR), mô hình thay thế, chính sách quyết định, ablation |
| `checks/` | Đặc tính bay để kiểm tra tham số và chọn lưới |
| `tests/` | `AircraftTest` (tính hợp lý vật lý), `SystemTest` (nguồn, gió, cấu hình, kịch bản) |

## Giới hạn

- [R37] không cho điểm trim hay cực số của Aerosonde, và ta không có dữ liệu bay để kiểm chứng. Các test chỉ kiểm tra tính hợp lý vật lý: cân bằng năng lượng, xu hướng theo khối lượng/CG, dấu của các mode.
- Các giả thiết GT được liệt kê trong `EXPERIMENTS.md` mục 1–3.
