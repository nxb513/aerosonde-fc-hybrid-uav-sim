# Thiết kế thí nghiệm — Aerosonde FC hybrid, bay vòng kín đến khi cạn nhiên liệu (E3)

**Nguyên tắc:**
- Mọi lựa chọn khó hoặc chưa có số liệu đều được **quét lưới** hoặc ghi rõ là giả thiết (GT).
- Số liệu có nguồn ghi mã [Rxx] (tài liệu đã đọc toàn văn).
- Dải tham số máy bay suy ra từ **đặc tính bay tính bằng chính mô hình** (`checks/flight_envelope.m`).
- Thiết kế cũ E1/E2 (quyết định một lần trước mỗi đoạn 45 s) **đã hủy**. Toàn bộ nằm trong thư mục rác `_rac/` (không thuộc repo) và không dùng cho kết quả nào.

## 1. Nền máy bay: Aerosonde [R37, App. E]

**Nguồn và độ tin cậy:**
- Tham số lấy từ Beard & McLain [R37, Bảng E.1–E.2]. [R37] cho biết bộ tham số gốc từ AeroSim Blockset (Unmanned Dynamics).
- Aerosonde thật: sải 2.9 m, rỗng khoảng 8 kg, MTOW 13–14 kg, động cơ xăng khoảng 1 kW. Đây là nguồn bách khoa, chỉ dùng để kiểm tra kích cỡ.

**Kiểm tra hợp lý** (mô hình, bus 42.9 V, CG tham chiếu):

| Đại lượng | Giá trị | Nhận xét |
|---|---|---|
| CL_max | 1.23 (α0 = 14°) | α0 = 27° của [R37] cho CL_max = 2.42, phi thực tế → GT α0 = 14° |
| Vận tốc stall | 16.2 / 17.6 / 18.9 / 20.1 m/s ở 11 / 13 / 15 / 17 kg | — |
| Vận tốc trim khả thi | 18–31 m/s (11 kg) … 22–30 m/s (17 kg) | → ứng viên V_a ∈ {22, 25, 28} |
| Công suất điện ở 25 m/s | 394–465 W | → FC 600 W |
| Góc leo tối đa ở 25 m/s | 6.5–11.4° | → ứng viên γ̄ ∈ {2, 5, 8}° |
| Bank quay vòng đều tối đa | 40–57° | → ứng viên φ̄ ∈ {15, 25, 35, 45}° |
| Dự trữ ổn định ở CG tham chiếu | 0.49 c̄ (rất cao) | → CG là biến thiết kế, dịch lùi 2–8 cm |
| Các mode | short-period −4.7 ± 9.7i, phugoid −0.04 ± 0.50i, roll −21, dutch-roll −1.4 ± 5.8i, spiral +0.05 | hợp lý |

**Giả thiết (GT):**
- Servo: [R37] không cho thông số. Dùng bậc 2, ωn = 100 rad/s, ζ = 0.707, trễ 0.07 s, tốc độ 200°/s, giới hạn góc 25°.
- Hệ số khí động: bất định 10% (1σ), vì [R37] không cho độ lệch chuẩn. Mỗi kịch bản lấy một mẫu (`ac_perturb`).
- Vận tốc nào không trim được (ví dụ 22 m/s với cấu hình nặng, CG lệch trước, elevator cần vượt 25°) thì bị loại khỏi tập ứng viên của chuyến bay đó.

## 2. Hệ nguồn

- **Kiến trúc:** bán chủ động [R3]: FC và siêu tụ qua DC/DC, pin nối thẳng bus. Kiến trúc thụ động [R5][R24] chỉ dùng cho ablation A9.
- **FC:** đường phân cực AeroStack [R24], co theo diện tích cell; trễ kích hoạt τ = Td/3 [R19]; giới hạn tốc độ tăng dòng 2 A/s cho stack 200 W [R25].
  - Hai cấu hình:
    - **nhẹ:** 600 W @ 458 W/kg [R3], bình 1 h, khoảng 13.7 kg;
    - **nặng:** 600 W @ 224 W/kg [R5], bình 2 h, khoảng 16.8 kg.
  - Td = 10 s (danh định). S1 xét Td = 1 và 20 s.
- **Bình H₂:** composite 1.4% H₂ theo khối lượng [R26]. Dung tích tính cho thời gian thiết kế ở 60% P_fc, hiệu suất 45% (GT).
  - **Mức nạp ban đầu** {25, 50, 100}% dung tích; khối lượng bình giữ nguyên.
- **Pin:** LiFePO₄ 13S (42.9 V) [R4]; lão hóa [R18].
- **Siêu tụ:** Maxwell 58 F 16 V [R3], mô hình C–ESR. A5 bỏ siêu tụ.
- **EMS lọc tần số** [R3]: FC nhận thành phần tần số thấp, siêu tụ nhận phần cao, pin bù phần còn lại.
  - **Mức bù** = tần số cắt f_c ∈ {0.06, 0.3, 1} Hz [R3][R15], do planner chọn. f_c thấp nghĩa là pin/siêu tụ gánh nhiều hơn, FC chạy mượt hơn.
  - Baseline: f_c thích nghi theo SoC siêu tụ [R3], luật [R27], ECMS [R30].

## 3. Gió (không cần địa điểm; "mùa" = mức gió)

- **Mức gió** W̄ ∈ {0, 4, 8, 12} m/s, tức 0–50% vận tốc hành trình (MAV thường gặp gió 20–50% vận tốc [R37, mục 2.4]).
- **Gió trung bình theo nhiệm vụ** (GT):
  - Lấy trong U[W̄ − 2, W̄ + 2] (không âm). Hướng trôi U[−30°, 30°] giữa hai nhiệm vụ.
  - Chuyển dần với hằng số thời gian 20 s.
- **Nhiễu Dryden tầng thấp** [R12]: hệ số cường độ U[0.5, 3] (GT); nhân đôi trong nhiệm vụ "giữ quỹ đạo trong gió giật".
- **Hệ quả:** không dùng dữ liệu gió thật (ERA5), nên **không tuyên bố "gió thật"** khi viết bài.

## 4. Nhiệm vụ và planner

**Chuỗi nhiệm vụ** (`flight/task_specs`, `task_make`): ngẫu nhiên theo seed, giống nhau với mọi chính sách. λ và hạn hoàn thành là GT.

| Loại | Hình học | λ | Hạn = T_ref × |
|---|---|---|---|
| Chuyển chặng | waypoint 1–2.5 km, lệch hướng ±120° | 0.2 | 2.0 |
| Đổi hướng | 90/135/180° rồi bay thẳng 400 m | 0.5 | 1.5 |
| Leo/hạ | 20–60 m (giữ trong 60–250 m) | 0.5 | 1.5 |
| Bay vòng chờ | 2 vòng, bán kính 120–250 m | 0.8 | 1.25 |
| Quét luống cày | 4 đường dài 400–700 m, cách nhau 80–160 m | 0.8 | 1.25 |
| Giữ quỹ đạo trong gió giật | thẳng 1.5 km, nhiễu ×2 | 0.8 | 1.25 |

- **T_ref:** độ dài đường bay / 25 m/s; các cung quay tính với bán kính ứng với bank 30°; leo/hạ ở 5°.
- **Bỏ nhiệm vụ:** nếu quá max(3 T_ref, T_ref + 120 s).
- **Bám đường** theo trường vector [R37, Eq. 10.9, 10.15], roll feed-forward [R37, Eq. 10.18], chuyển chặng bằng nửa mặt phẳng [R37, ch. 11].
- **Autopilot:** successive loop closure [R37, ch. 6] cộng yaw damper. Gain thiết kế ở 22/25/28 m/s; feed-forward trim nội suy theo vận tốc đo.

**Planner** (`flight/planner_decide`), MPC cuốn chiếu trên tập rời rạc:
- **Khi nào quyết định:** mỗi 3 s và đầu mỗi nhiệm vụ. Chạy lại planner định kỳ là cách chuẩn để bù sai số mô hình [R37, ch. 12].
- **Ứng viên:** 36 quỹ đạo (φ̄, γ̄, V_a) × 3 mức bù, với dự báo cho chân trời 10 s.
- **Ràng buộc:**
  - cận trên AFP (conformal) ≤ 1;
  - SoC dự báo cuối chân trời ≥ 35%.
- **Hàm chi phí:** J = λ·τ + (1 − λ)·(e + d), trong đó:
  - τ = 1 − tốc độ tiến độ / 25 m/s;
  - e = H₂ tương đương, quy đổi năng lượng pin/siêu tụ theo hệ số s_eq phụ thuộc SoC, dạng ECMS [R30]; s0 = 2.5 và k = 3 là GT;
  - d = suy giảm pin [R18] cộng biến thiên công suất FC [R16][R17], chuẩn hóa theo trung vị.

**Điều kiện dừng chuyến bay:**
- hết H₂;
- SoC < 30% (dự trữ hạ cánh [R27]);
- rơi (h < 5 m);
- hoặc quá 4 h.

**Nhãn vi phạm** của một cửa sổ hay một nhiệm vụ: m = max(lệnh lái / giới hạn góc, α_p99 / α_lim, thời gian bão hòa tốc độ lái liên tục / 0.2 s).
- Vi phạm khi m > 1. Bão hòa tốc độ lái kéo dài quá 0.2 s là GT. α_lim = α0 − 2°.
- **Nhiệm vụ đạt** = hoàn thành trước hạn và m của cả nhiệm vụ ≤ 1.

**Bước tích phân** 0.01 s (RK4).
- Kiểm tra ở gió lặng: so với 0.0025 s, H₂, SoC và thời gian nhiệm vụ lệch dưới 0.5%, và phân loại vi phạm giống nhau.
- Ở gió có nhiễu, chuỗi nhiễu phụ thuộc bước tích phân, nên chỉ so được ở mức thống kê.

## 5. Thí nghiệm

| | E3X: thăm dò (huấn luyện) | E3: đánh giá |
|---|---|---|
| Số chuyến bay | 2000 × 10 phút | 144 kịch bản × 17 chính sách + 192 (S1, S2) = 2640 |
| Chính sách | ứng viên ngẫu nhiên, giữ 10 s | xem mục 6 |
| Cấu hình | nhẹ/nặng; CG U[−8, 0] cm; Td {1, 10, 20} s; 20% không siêu tụ; 15% thụ động | nhẹ/nặng × CG {−6, −2} cm |
| Gió | W̄ U[0, 12] | W̄ {0, 4, 8, 12} |
| Nhiên liệu, trạng thái đầu | SoC U[0.35, 0.95], siêu tụ U[0.3, 1] | mức nạp {25, 50, 100}%; SoC U[0.5, 0.9], siêu tụ U[0.3, 1] |
| Seed | mỗi chuyến một seed | 3 (cùng nhiệm vụ, gió, khí động cho mọi chính sách) |

**Huấn luyện** (`ai/e3_train_all`, chạy trên GitHub ở đầu stage eval):
- **Dữ liệu:** mỗi cửa sổ 10 s của E3X là một mẫu, 33 đặc trưng (`flight/e3_features`).
- **Chia dữ liệu:** theo chuyến bay, 60/20/20 cho huấn luyện / hiệu chuẩn / kiểm tra.
- **AFP:** rừng hồi quy lượng tử [R39] τ = 0.9 cộng CQR một phía [R32], α = 0.1, hiệu chuẩn theo nhóm mức gió.
- **Mô hình thay thế:** tiến độ, công suất đỉnh, H₂, công suất pin + siêu tụ, suy giảm.
- **Learning curve** (`ai/e3_learning_curve`):
  - Huấn luyện AFP trên 25 → 1200 chuyến bay theo thứ tự thu dữ liệu; tập hiệu chuẩn và tập kiểm tra cố định.
  - Thêm 3 tập con ngẫu nhiên cho mỗi mốc. Mỗi mốc lưu một checkpoint mô hình.
  - Vẽ pinball loss [R39, Eq. 3], độ phủ, recall, báo động giả và AUC; thêm lỗi OOB theo số cây.

**Thước đo** (`ai/e3_analysis`):
- **Chính:** số nhiệm vụ đạt trước khi cạn nhiên liệu.
- **Phụ:**
  - H₂ tương đương (đã hiệu chỉnh SoC) trên mỗi nhiệm vụ đạt;
  - số nhiệm vụ vi phạm, thời gian vi phạm, tỉ lệ rơi;
  - thời gian bay, SoC cuối, suy giảm FC (μV/cell) và pin (%).
- **So sánh:** theo cặp với chính sách của ta trên cùng kịch bản, dùng kiểm định Wilcoxon.

**Kiểm tra AFP trên quỹ đạo của chính sách (E3P)** (`ai/e3_onpolicy`):
- Sau mỗi quyết định, đo m thực tế trên **10 s** tiếp theo (đúng chân trời mà cận trên U dự đoán), trong khi planner vẫn quyết định lại mỗi 3 s.
- So với U và lượng tử thô q của ứng viên đã chọn.
- Báo cáo:
  - độ phủ P(m ≤ U), toàn bộ và theo nhóm gió;
  - tỉ lệ vi phạm khi U ≤ 1;
  - ước lượng độ phủ nếu hiệu chuẩn lại bằng dữ liệu của chính sách (hiệu chuẩn trên seed 1–2, kiểm tra trên seed 3).
- **Lưới E3P:** chạy lại đúng các chuyến E3 của ours, A1, A2, A10 (576 chuyến) có thêm log này. Log không làm đổi cách bay hay cách quyết định, nên các chuyến ours trong E3P phải ra đúng kết quả của E3.
- Ghi log 3 s trong E3 (`trace.decisions`) chỉ để tham khảo, vì không cùng chân trời với U.

## 6. Chính sách, ablation, độ nhạy

| Mã | Mô tả |
|---|---|
| ours | AFP + conformal theo nhóm gió, chọn quỹ đạo + mức bù, λ theo nhiệm vụ |
| A1 | Không ràng buộc AFP |
| A2 | Lượng tử thô, không conformal |
| A3 | Mô hình không có đặc trưng cấu hình (khối lượng, CG, …) |
| A4 | Mô hình không có đặc trưng gió |
| A5 | Không siêu tụ (phần cứng) |
| A6 | Chỉ đổi quỹ đạo: FC theo tải, loại quỹ đạo cần pin gánh đỉnh công suất |
| A7 | Chỉ bù năng lượng: giữ quỹ đạo danh nghĩa (35°, 5°, 25 m/s), chọn mức bù |
| A8 | λ = 0.5 cho mọi nhiệm vụ |
| A9 | Kiến trúc thụ động (phần cứng) |
| A10 | Conformal toàn cục thay vì theo nhóm |
| A11 | Mô hình không có trạng thái điện (SoC, siêu tụ, P_FC, U_bus) |
| fixed_gentle / nominal / aggr | Quỹ đạo cố định (15°, 2°) / (35°, 5°) / (45°, 8°) ở 25 m/s, EMS [R3] |
| nominal_rule / nominal_ecms | Quỹ đạo danh nghĩa, EMS luật [R27] / ECMS [R30] |
| S1 | Td = 1 / 20 s (48 kịch bản) |
| S2 | Chu kỳ planner 1 / 5 s (48 kịch bản) |

## 7. Hạ tầng

- Repo public; workflow `.github/workflows/e3.yml` chạy tối đa 20 job song song.
  - `stage=explore`: 20 job E3X, gộp thành `E3X-all`.
  - `stage=eval`, `explore_run=<run id>`:
    - huấn luyện (`E3-models`, `E3X-report`);
    - 60 job E3 (20 job một lúc);
    - gộp và phân tích (`E3-all`).
  - `stage=onpolicy`, `models_run=<run id của eval>`: 20 job E3P, dùng lại `E3-models` (không huấn luyện lại), gộp và phân tích (`E3P-all`).
  - `stage=test`: 6 chuyến bay ngắn.
- **Checkpoint:**
  - `ckpt_<job>.mat` ghi sau mỗi chuyến bay;
  - `fckpt_<job>.mat` ghi trạng thái chuyến đang bay mỗi 5 phút tính;
  - cả hai luôn được tải lên. Khi chạy lại job lỗi, job tự lấy checkpoint của mình và chạy tiếp.
