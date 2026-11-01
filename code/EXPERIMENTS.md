# Kế hoạch thí nghiệm (chốt 2026-11-01)

Nguyên tắc: mọi lựa chọn khó hoặc chưa có số liệu đều được **quét lưới (grid)** thay vì chọn một giá trị, và được ghi rõ là giả thiết (GT). Số liệu có nguồn ghi mã [Rxx] theo `references_read.bib`.

## 1. Cấu hình phần cứng (chốt)

**Kiến trúc nguồn:**
- **Bán chủ động (semi-active)**, dùng làm kiến trúc chính [R3]: pin nối thẳng bus; FC và siêu tụ đi qua DC-DC có lệnh công suất.
- **Thụ động (passive)**, dùng làm baseline [R5][R24]: FC, pin và siêu tụ nối song song, chia dòng theo điện áp.

**Pin nhiên liệu:**
- Đường phân cực AeroStack đo thực [R24, Eq. 1]: V = V0 − B·ln(I)·[lọc bậc 1] − R·I − m·e^{nI}.
- Phần kích hoạt được lọc với τ = Td/3 [R19].
- Định cỡ theo công suất P_fc ∈ {150, 200, 250} W:
  - dòng được co giãn theo diện tích cell;
  - khối lượng FC + bộ điều khiển theo mốc 224 W/kg (FLY-150: 150 W / 0.67 kg [R5]).

**Bình H₂:**
- Metal hydride 1.09 kg [R5].
- Tiêu thụ H₂ = N·I/(2F), chia cho hiệu suất sử dụng nhiên liệu η_F = 0.9 khi P > 50 W [R24].

**Động học FC (lưới):**
- Td ∈ {1, 10, 20} s: EPAC-500 đo 1 s [R19]; FC UAV tự tạo ẩm 10–20 s [R24].
- Giới hạn tốc độ tăng dòng ∈ {2, 5} A/s: [R25] ghi nhận FC cần > 5 s để lên 10 A.

**Pin:**
- Cell LiFePO₄ 3.3 V 2.3 Ah, mô hình Tremblay–Dessaint với tham số Bảng 1 của [R4].
- Lão hóa theo [R18], cùng hóa học graphite–LiFePO₄.
- Cấu hình 5S1P hoặc 5S2P.
- Khối lượng pack theo mốc 65 Wh/kg, lấy từ pack 4S 1500 mAh 343 g [R5] (GT).

**Siêu tụ:**
- Yunasko 25 V, 7 F, 0.6 Wh, 279 g [R5], qua DC-DC.
- Mô hình C–ESR, với ESR ∈ {10, 30} mΩ (GT). Đây là xấp xỉ của Zubieta [R20].
- Giới hạn e_sc ≥ 25 % [R3].
- Có hoặc không có siêu tụ (ablation).

**DC-DC:** hiệu suất η_dc ∈ {0.93, 0.97} (GT).

**Khối lượng và CG:**
- Tổng khối lượng thêm vào tính từ các linh kiện trên.
- Tháo pin gốc X8 ∈ {0, 0.5} kg (GT).
- Dịch CG ròng Δx_cg ∈ {−3, −1.5, 0, +1.5, +3} cm, đặt linh kiện để đạt giá trị này.
- Quán tính tính theo chất điểm [x8_apply_config].

**Hệ đẩy:**
- Hai bộ tham số động cơ [R2] / [R23] dùng làm bất định.
- Hệ số khí động ~ N(μ, σ) theo [R2, Bảng 8].

## 2. Gió ("mùa" = mức gió, không cần địa điểm)

- **Gió trung bình** W20 ∈ {2, 6, 10, 14} m/s, tương ứng lặng / nhẹ / mạnh / rất mạnh.
  - DS1 ghi nhận khoảng 9–10 m/s [R2].
  - Hướng gió được lấy mẫu đều.
- **Nhiễu Dryden tầng thấp** [R12]: σ_w = 0.1·u20, σ_u/σ_w = 1/(0.177 + 0.000823h)^0.4, L theo h.
- **Hệ số cường độ nhiễu** ∈ {0.5, 1, 2} × (GT; MIL dùng xác suất vượt ngưỡng).
- **Profil gió log** từ 20 ft lên độ cao bay [R12].
- ERA5 cho một địa điểm cụ thể có thể cắm vào sau, thay lưới W20 bằng phân bố thật.

## 3. Nhiệm vụ và quỹ đạo ứng viên

- **Đoạn nhiệm vụ chuẩn (segment):** bay thẳng ở trim → quay Δχ ∈ {90°, 180°} và leo Δh ∈ {0, 30, 60} m → giữ. Mỗi đoạn dài 40 s.
- **Ứng viên quỹ đạo k:** bank tối đa φ̄ ∈ {15°, 25°, 35°, 45°} × tốc độ leo γ̄ ∈ {3°, 6°, 9°} × V_a ∈ {17, 20, 23} m/s.
  - Bank lớn, leo dốc = **"giữ quỹ đạo gắt + bù năng lượng"**.
  - Bank nhỏ, leo thoải = **"đổi quỹ đạo"**: đường dài hơn, ít đỉnh công suất hơn.
  - Quan hệ bán kính – bank theo [R37, Eq. 5.16]; Dubins [R37, ch. 11].
- **Autopilot:** successive loop closure [R37, ch. 6], độ lợi tính từ mô hình tuyến tính tại trim của từng cấu hình (a_φ, a_θ, a_V). Có bão hòa φ_c ≤ φ̄, θ_c, ga [0, 1] và anti-windup.

## 4. Các thí nghiệm

**E1 — dữ liệu AFP (lấy mẫu ngẫu nhiên):**
- Lấy mẫu trên các khoảng liên tục: khối lượng thêm [0, 2.5] kg, Δx_cg [−4, 4] cm, các ứng viên, gió, nhiễu, hệ số khí động, seed.
- **Nhãn** = biên cơ cấu chấp hành m = max_t max(|δ|/30°, |δ̇|/200°/s), cùng α_max, cờ bão hòa, năng lượng và đỉnh công suất.
- Khoảng 20 job × N mẫu.

**E2 — lưới có cấu trúc (kiểm thử + quyết định):**
- Cấu hình chốt × mức gió × nhiễu × nhiệm vụ × ứng viên × seed.
- Mô phỏng 6-DOF có hệ nguồn trong vòng kín (EMS mặc định: lọc tần số).
- Lưu chuỗi P_load(t) để đánh giá lại các EMS ngoài vòng: rule [R27], lọc tần số [R3], ECMS [R30], thụ động [R5].
  - GT: tải công suất phụ thuộc yếu vào điện áp bus khi autopilot giữ V_a.

**Đánh giá:**
- **AFP:** hồi quy lượng tử bằng rừng ngẫu nhiên, hiệu chuẩn conformal (CQR) [R32].
  - Hiệu chuẩn theo nhóm mức gió.
  - Báo cáo độ phủ theo nhóm.
  - Số mẫu theo cận Tempo [R21].
- **Quyết định (ECP + planner receding-horizon trên tập ứng viên):**
  - Chi phí J_k(λ) = w_t·thời gian + w_H₂·H₂ + w_fc·suy giảm FC [R16][R17] + w_bat·Q_loss [R18], với ràng buộc P(m > 1) ≤ δ.
  - Oracle = argmin trên kết quả Monte Carlo.
  - Chính sách AI = AFP + mô hình thay thế (surrogate) năng lượng, chọn k không cần mô phỏng.
  - Nhiệm vụ nhiều đoạn nối tiếp, SoC mang sang đoạn sau, để thể hiện tính "liên tục".
- **Baseline:** luôn gắt + bù; luôn thoải; EMS-only (rule / lọc / ECMS / thụ động); planner bỏ qua cơ cấu chấp hành.

**Ablation (bắt buộc):**

| Mã | Bỏ / đổi | Kiểm tra điều gì |
|---|---|---|
| A1 | Không có AFP (chọn theo năng lượng) | Giá trị của dự báo vi phạm |
| A2 | AFP không conformal (lượng tử thô) | Giá trị của bảo đảm xác suất |
| A3 | AFP không có đặc trưng m, CG, J | Vai trò của cấu hình khối lượng/CG |
| A4 | AFP không có đặc trưng gió | Vai trò của gió |
| A5 | Không có siêu tụ | Vai trò của siêu tụ khi FC chậm |
| A6 | Chỉ đổi quỹ đạo (không bù) | Giá trị của bù năng lượng |
| A7 | Chỉ bù (không đổi quỹ đạo) | Giá trị của đổi quỹ đạo |
| A8 | λ cố định / λ theo nhiệm vụ | Vai trò của tham số đánh đổi |
| A9 | Kiến trúc thụ động | Vai trò của EMS chủ động |
| A10 | Hiệu chuẩn conformal toàn cục / theo nhóm gió | Độ phủ theo nhóm |
| S1 | Td = 1 / 10 / 20 s (độ nhạy) | Động học FC quyết định nhu cầu bù |

## 5. Hạ tầng chạy

- Repo GitHub public; workflow `grid.yml` chạy ma trận 20 job song song (max-parallel 20).
- Mỗi job chạy một phần (chunk) của lưới, rồi upload artifact; job `gather` gộp kết quả.
- Lưới lớn được chia thành nhiều lượt chạy.
- Thời gian chạy: dt = 5 ms cho Monte Carlo (đã kiểm tra so với 2 ms trong test).
