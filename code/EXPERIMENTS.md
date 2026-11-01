# Kế hoạch thí nghiệm — Aerosonde chuyển đổi sang FC hybrid (bản mới, thay toàn bộ bản X8)

**Nguyên tắc:**
- Mọi lựa chọn khó hoặc chưa có số liệu đều được **quét lưới** và ghi rõ là giả thiết (GT).
- Số liệu có nguồn ghi mã [Rxx].
- Mọi dải tham số đều suy ra từ **đặc tính bay tính bằng chính mô hình** (`checks/flight_envelope.m`, kết quả trong `results/flight_envelope.md`).
- Bản X8 trước đây đã **hủy**: X8 (3.4 kg) không chở nổi hệ FC.

## 1. Nền máy bay: Aerosonde [R37, App. E]

**Nguồn và độ tin cậy:**
- Tham số lấy từ Beard & McLain [R37, Bảng E.1–E.2]. [R37] cho biết bộ tham số gốc từ AeroSim Blockset (Unmanned Dynamics); mô hình Aerosonde AeroSim cũng được Samar, Gorinevsky & Boyd (Stanford) dùng.
- Aerosonde thật: sải 2.9 m, rỗng khoảng 8 kg, MTOW 13–14 kg, động cơ xăng khoảng 1 kW. Đây là nguồn bách khoa, chỉ dùng để kiểm tra kích cỡ.

**Kiểm tra hợp lý** (mô hình, bus 42.9 V, CG tham chiếu):

| Đại lượng | Giá trị | Nhận xét |
|---|---|---|
| CL_max | 1.23 (α0 = 14°) | α0 = 27° của [R37] cho CL_max = 2.42, phi thực tế → GT: α0 ∈ [12, 16]° |
| Vận tốc stall | 16.2 / 17.6 / 18.9 / 20.1 m/s ở 11 / 13 / 15 / 17 kg | — |
| Vận tốc trim khả thi | 18–31 m/s (11 kg) … 22–30 m/s (17 kg) | — |
| Công suất điện ở 25 m/s | 394–465 W | → định cỡ FC 400–800 W |
| Góc leo tối đa ở 25 m/s | 6.5–11.4° | → lưới góc leo |
| Bank quay vòng đều tối đa | 40–57° | → lưới bank |
| Dự trữ ổn định ở CG tham chiếu | 0.49 c̄ | rất cao: elevator trim −10° đến −16° ở 25 m/s, tới −21° ở 20 m/s, 13 kg |
| Các mode | short-period −4.7 ± 9.7i, phugoid −0.04 ± 0.50i, roll −21, dutch-roll −1.4 ± 5.8i, spiral +0.05 (hơi mất ổn định, đúng [R37]) | hợp lý |

- **Hệ quả của dự trữ ổn định cao:** CG là biến thiết kế của bộ chuyển đổi. Dịch lùi 2–8 cm cho dự trữ ổn định 38–7% c̄.
- **GT về servo:** [R37] không cho thông số servo. Dùng ωn = 100 rad/s, ζ = 0.707, trễ 0.07 s [R2] và tốc độ 200°/s [R7]; giới hạn góc 25°. Tất cả quét lưới trong E1.
- **GT về hệ số khí động:** bất định 10% (1σ), vì [R37] không cho độ lệch chuẩn.

## 2. Hệ nguồn

- **Kiến trúc:** bán chủ động [R3] (chính); thụ động [R5][R24] (ablation).
- **FC:** đường phân cực AeroStack [R24], co theo diện tích cell; τ = Td/3 [R19].
  - P_fc ∈ {400, 600, 800} W.
  - Công suất riêng {224 [R5], 458 [R3]} W/kg.
  - Td ∈ {1, 10, 20} s; giới hạn tốc độ tăng dòng {2, 5} A/s (theo stack 200 W) [R25].
- **Bình H₂:** composite 1.4% H₂ theo khối lượng [R26]; lượng H₂ cho {1, 2} h ở 60% P_fc với hiệu suất 45% (GT).
- **Pin:** LiFePO₄ 13S (42.9 V, gần V_max 44.4 V của động cơ [R37]), 1P hoặc 2P [R4]; lão hóa theo [R18].
- **Siêu tụ:** {không, Yunasko 7 F 25 V [R5], Maxwell 58 F 16 V [R3]}.
- **Khối lượng:** tháo động cơ xăng + nhiên liệu m_remove ∈ [1.5, 3] kg (GT).
  - Cấu hình E2 "nhẹ" (600 W @ 458 W/kg, 1 h): khoảng 13.7 kg.
  - Cấu hình E2 "nặng" (600 W @ 224 W/kg, 2 h): khoảng 16.8 kg.

## 3. Gió (không cần địa điểm; "mùa" = mức gió)

- Gió trung bình W_h ∈ [0, 12] m/s, tức 0–50% vận tốc hành trình (MAV thường gặp gió 20–50% vận tốc [R37, mục 2.4]).
- Nhiễu Dryden tầng thấp [R12] với hệ số cường độ ∈ [0.5, 3] (GT).
- Độ cao bay 100 m.

## 4. Nhiệm vụ và quỹ đạo ứng viên

- **Đoạn nhiệm vụ:** bay thẳng ở trim → đổi hướng Δχ và leo Δh → giữ. Thời lượng 45 s.
- **Ứng viên:** bank tối đa φ̄, góc leo tối đa γ̄, vận tốc V_a.
  - Bank lớn + leo dốc = "giữ quỹ đạo gắt, để pin/siêu tụ bù".
  - Bank nhỏ + leo thoải = "đổi quỹ đạo".
- **Autopilot:** successive loop closure [R37, ch. 6] cùng yaw damper washout. Vòng pitch dùng thiết kế bền theo CG và trễ servo (GT).

## 5. Lưới thí nghiệm

| | E1 (ngẫu nhiên, 10 000) | E2 (có cấu trúc, 20 736) |
|---|---|---|
| FC | 400/600/800 W; 224/458 W/kg; 1/2 h | nhẹ / nặng (xem mục 2) |
| Pin, siêu tụ, Td, ramp | 1P/2P; 0/1/2; 1/10/20 s; 2/5 A/s | 1P; Maxwell; 10 s; 2 A/s |
| m_remove, dx_cg | U[1.5, 3] kg; U[−8, 0] cm | 2.5 kg; {−6, −4, −2} cm |
| α0, δ_max, tốc độ servo, trễ | U[12, 16]°; U[20, 30]°; U[100, 250]°/s; U[0.03, 0.08] s | 14°; 25°; 200°/s; 0.07 s |
| Gió W_h, nhiễu | U[0, 12]; U[0.5, 3] | {0, 4, 8, 12}; {1, 2.5} |
| Δχ, Δh | {45, 90, 135, 180}°; U[0, 60] m | {90, 180}°; {0, 50} m |
| φ̄, γ̄, V_a | U[15, 50]°; U[1, 10]°; U[19, 30] m/s | {15, 25, 35, 45}°; {2, 5, 8}°; {22, 25, 28} m/s |
| Seed | — | 3 (cùng gió/khí động cho mọi ứng viên) |

## 6. Đánh giá và ablation

- **AFP:** rừng hồi quy lượng tử + CQR [R32]; độ phủ theo nhóm gió; cận số mẫu Tempo [R21].
- **Quyết định:**
  - J = λ·thời gian + (1 − λ)·(năng lượng + suy giảm [R16][R17][R18]), với λ ∈ {0.2, 0.5, 0.8}.
  - So với oracle và các baseline.
- **Ablation:**

| Mã | Bỏ / đổi |
|---|---|
| A1 | Không AFP |
| A2 | Không conformal |
| A3 | Không đặc trưng cấu hình |
| A4 | Không đặc trưng gió |
| A5 | Không siêu tụ |
| A6 | Chỉ đổi quỹ đạo |
| A7 | Chỉ bù năng lượng |
| A8 | λ cố định |
| A9 | Kiến trúc thụ động |
| A10 | Conformal theo nhóm |
| S1 | Td = 1 / 20 s (độ nhạy) |

- **EMS so sánh:** rule [R27], lọc tần số [R3], ECMS [R30].

## 7. Hạ tầng

- Repo public, workflow `grid.yml` chạy 20 job song song.
- Phân tích: `ai/run_analysis(dirE1, dirE2)`.
