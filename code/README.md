# Mô hình Skywalker X8 6-DOF (MATLAB) — bước 1

Mô hình bay của UAV cánh bay X8. Lực đẩy lấy điện áp bus DC làm đầu vào, đây là điểm ghép với hệ nguồn FC/pin/siêu tụ (bước sau). Code chỉ dùng MATLAB thuần cộng `fsolve` (Optimization Toolbox), không cần Simulink, nên chạy được trên GitHub Actions.

## Chạy

```matlab
cd code
run_tests                     % 10 test kiểm chứng (test DS1 cần data/DS1)
addpath examples; demo_trim_margin   % biên elevon theo khối lượng/CG
```

## Cấu trúc

| Thư mục / file | Nội dung | Nguồn |
|---|---|---|
| `params/x8_params.m` | Khối lượng, quán tính, hình học; hệ số khí động **kèm độ lệch chuẩn** (cho Monte Carlo); cánh quạt, động cơ, servo | [R2] Bảng 1, 6–9; [R7] giới hạn elevon; [R1] vùng α hợp lệ |
| `params/x8_apply_config.m` | Lắp/tháo linh kiện → m, CG, J (định lý trục song song) | cơ học vật rắn |
| `model/x8_aero.m` | C_L, C_D (có hạng C_T), C_m, C_Y, C_l, C_n; chuyển stability → thân | [R2, Eq. 15–21], [R37, Eq. 4.25] |
| `model/x8_propulsion.m` | Cánh quạt + động cơ DC; U = δt·U_bus; P_el, I_bus | [R2, Eq. 5–9, 28–29] |
| `model/x8_forces_moments.m` | Tổng lực/mô-men quanh CG thực tế; mô-men cánh quạt đẩy sau + con quay; chuyển mô-men khi CG lệch | [R2, Eq. 11–13, 22], [R1, Eq. 18] |
| `model/x8_derivatives.m` | 6-DOF quaternion + Ω_p + servo (19 trạng thái) | [R2, Eq. 23–26], [R37, ch. 3, App. B] |
| `model/x8_actuator_deriv.m` | Elevon bậc 2 có bão hòa biên độ/tốc độ; ga bậc 1 | [R2, Bảng 9], [R7] |
| `sim/x8_simulate.m` | RK4 bước cố định, trễ lệnh, gió, cờ bão hòa (nhãn cho AFP), năng lượng điện | — |
| `trim/x8_trim.m` | Trim bay thẳng hoặc quay vòng; kiểm tra khả thi elevon/ga/α; biên elevon còn lại | [R37, mục 5.3, App. F] |
| `trim/x8_linearize.m` | Jacobian số, tách mode dọc/ngang | [R37, App. F.1], [R2, Eq. 39–42] |
| `tests/X8ModelTest.m` | Kiểm chứng | xem bên dưới |

Trạng thái (19): vị trí NED, vận tốc đất trong hệ thân, quaternion, (p, q, r), Ω_p, góc elevon phải/trái, tốc độ elevon, ga thực tế. Lệnh: `[δer_c; δel_c; δt_c]`.

## Kiểm chứng (đã chạy, 10/10 test đạt)

**Tuyến tính hóa tại điểm hoạt động của [R2, Eq. 36–38]** (Va = 17.9 m/s, gió thân [−5.93, −6.62, −2.75] m/s):

| Mode | Mô hình này | [R2] công bố |
|---|---|---|
| Short-period | −6.36 ± 4.89i | −6.19 ± 4.93i |
| Phugoid | −0.23 ± 0.98i | −0.22 ± 0.97i |
| Dutch-roll | −0.87 ± 3.77i | −0.84 ± 3.73i |
| Roll | −18.29 | −17.50 |
| Spiral | −0.41 | −0.42 |

Các phần tử A_lon và A_lat lệch tối đa khoảng 0.15 và 0.8 (đa số lệch 2–5 %). Sai khác còn lại có thể do ρ và Ω_p tại điểm đó, vì [R2] không công bố hai giá trị này.

**Trim tại cùng điều kiện** (γ_a = θ − α = −9.01°): α = 7.64° (R2: 7.9°), δe = −1.98° (R2: −2.35°), δt = 0.456 (R2: 0.44).

**Các test khác:**
- Trim được giữ khi mô phỏng 5 s (sai lệch V_a < 1e-3 m/s).
- Elevon không vượt ±30° và ±200°/s; trễ 0.07 s được tôn trọng.
- Phép lắp rồi tháo cùng một khối lượng trả về đúng cấu hình gốc.
- CG dịch về phía mũi thì cần elevon ngẩng lên nhiều hơn và biên elevon giảm.
- Nặng hơn thì α và công suất tăng.
- U_bus thấp hơn thì ga cao hơn nhưng công suất điện không đổi.

## Kiểm chứng với dữ liệu bay thật [DS1]

- **Dữ liệu:** `data/DS1/` gồm 17 maneuver X8, mỗi cái khoảng 10 s, 40 Hz, gió Tây Bắc mạnh. Nguồn: Løw-Hansen & Hann, DataverseNO 2024, doi:10.18710/U4TLYV, CC0.
- **Cách chạy:** `addpath validation; run_ds1_validation` (thêm đối số ρ nếu muốn đổi mật độ khí).
- **Cách làm:** mô phỏng **hở vòng** từ trạng thái EKF ban đầu, với lệnh, gió và điện áp pin đo được.
- **Kết quả (ρ = 1.225, trung vị trên 17 maneuver):**
  - RMSE: α 1.6°, θ 3.3°, φ 3.6°.
  - Tương quan trên maneuver dọc: α 0.90, q 0.95.
  - Tương quan trên maneuver ngang: p 0.95, r 0.90.
- **Dòng pin trung bình:** mô hình thấp hơn đo khoảng 5–6 % (theo từng maneuver từ −23 % đến +17 %).
  - Mô hình **không bắt được các đỉnh dòng ngắn** (đo lên tới 14 A, mô hình khoảng 9 A).
  - Với ρ = 1.12 (tính từ cảm biến nhiệt độ 30–43 °C, nhiều khả năng đặt trong thân máy bay) thì độ lệch tăng lên −13 %, nên giữ ρ = 1.225.
  - Dòng đo có thể gồm cả tải điện tử; README của [DS1] không nói rõ.
- **Hình:** `results/ds1_longitudinal_3211_5_rho1.225.png` và `results/ds1_lateral_121_4_rho1.225.png`. Bảng đầy đủ: `results/ds1_validation_rho*.csv`.
- **Lưu ý dữ liệu:** cột GPS_lon/lat thực tế tính bằng rad, dù README của bộ dữ liệu ghi "degree".

## Giả thiết của ta (không có trong tài liệu, ghi rõ để nêu trong bài)

- ρ = 1.225 kg/m³ ([R2] không nêu); U_bus danh định 15.8 V ([R2] ghi nhận dải 15.2–16.4 V).
- Kẹp hệ số tiến J ≤ 1, vì đa thức của [R2] ngoài vùng khớp không đảm bảo. ESC không hồi sinh (I_m ≥ 0), không tổn hao.
- Cách hiện thực bão hòa trong servo bậc 2 (chặn đạo hàm khi chạm giới hạn).
- Đường lực đẩy đi qua gốc hệ thân. Vận tốc khí tại điểm tham chiếu ≈ tại CG. Linh kiện lắp thêm coi là chất điểm nếu không cho J.

## Lưu ý mô hình

- Không có mô hình stall; vùng tin cậy α ∈ [−10°, 12°] [R1]. Trim ngoài vùng này bị đánh dấu `ok_alpha = false`.
- Ở 18 m/s, công suất điện khi bay ngang theo mô hình là khoảng 195 W (L/D ≈ 4.5, do C_D0 = 0.058 của [R2]).
  - [DS1] không có đoạn bay ngang so với khối khí để đối chiếu trực tiếp con số này: luồng khí đi lên nên γ_a ≈ −9°, và pin đo 89–168 W.
  - Mô hình hệ đẩy (dòng theo ga, Va, U) thì đã được kiểm chứng như mục trên.
- Đỉnh dòng ngắn bị dự báo thấp. Đây là điểm cần lưu ý cho siêu tụ, vì siêu tụ chính là nguồn xử lý các đỉnh này.
- Mô phỏng 5 s mất khoảng 0.8 s (dt = 2 ms).
