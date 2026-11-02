# Aerosonde FC-hybrid UAV: lập lịch quỹ đạo vòng kín có giới hạn cơ cấu chấp hành (MATLAB)

Nền máy bay là **Aerosonde** [R37, App. E], chuyển từ động cơ xăng sang hệ nguồn lai **pin nhiên liệu – pin – siêu tụ**.

Máy bay bay chuỗi nhiệm vụ đến khi hết H₂ hoặc SoC < 30%. Planner quyết định lại mỗi 3 s: chọn quỹ đạo (bank, góc leo, vận tốc) và mức bù năng lượng (tần số cắt EMS), dựa trên bộ dự báo vi phạm cơ cấu chấp hành có bảo đảm conformal.

Thiết kế thí nghiệm, nguồn tham số và các giả thiết (GT) nằm trong `EXPERIMENTS.md`.

## Chạy

```matlab
cd code
run_tests                                   % 22 test
setup_paths; r = run_flight(struct('policy', 'fixed_nominal', 'fill', 0.25));
flight_envelope([11 13 15 17], 42.9, 'results/flight_envelope.md')
```

Chạy lưới trên GitHub:
- Bước 1, thăm dò: `gh workflow run e3.yml -f stage=explore`.
- Bước 2, huấn luyện và đánh giá: `gh workflow run e3.yml -f stage=eval -f explore_run=<run id của bước 1>`.

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `params/` | `ac_params` (Aerosonde [R37], servo, α0); `ac_perturb` (bất định khí động); `ac_apply_config` (m, CG, J); `hybrid_config` (bố trí FC/bình/pin/siêu tụ) |
| `model/` | Khí động có stall và cản bậc hai [R37, Eq. 4.6–4.26]; động cơ + cánh quạt nối bus [R37, Eq. 4.17–4.21]; 6-DOF quaternion; servo bão hòa |
| `trim/` | Trim phối hợp (β = 0), tuyến tính hóa |
| `control/` | Autopilot [R37, ch. 6] + yaw damper; vòng pitch bền theo CG/trễ; roll feed-forward [R37, Eq. 10.18] |
| `power/` | FC [R24][R19], pin [R4][R18], siêu tụ, DC-DC, EMS (lọc tần số [R3] có mức bù chọn được, luật [R27], ECMS [R30]), kiến trúc thụ động |
| `wind/` | Dryden tầng thấp [R12], đổi cường độ giữa chuyến bay |
| `flight/` | Chuỗi nhiệm vụ, bám đường [R37, ch. 10–11], chuyến bay vòng kín (`run_flight`), planner và chính sách |
| `ai/` | Rừng cây đóng gói (dự báo nhanh, rừng hồi quy lượng tử [R39]), CQR [R32], huấn luyện, learning curve, phân tích E3 |
| `grid/` | Lưới E3X/E3/E3T, chia job có checkpoint, gộp kết quả |
| `checks/` | Đặc tính bay để kiểm tra tham số và chọn dải ứng viên |
| `tests/` | `AircraftTest` (tính hợp lý vật lý), `SystemTest` (nguồn, gió, cấu hình), `FlightTest` (nhiệm vụ, bám đường, dừng, checkpoint, rừng cây) |

## Giới hạn

- [R37] không cho điểm trim hay cực số của Aerosonde, và không có dữ liệu bay để kiểm chứng. Các test chỉ kiểm tra tính hợp lý vật lý.
- Gió là gió tổng hợp (mức gió + Dryden), không phải dữ liệu đo.
- Các giả thiết GT được liệt kê trong `EXPERIMENTS.md`.
