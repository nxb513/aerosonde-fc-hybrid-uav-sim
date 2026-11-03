# Thiết kế thí nghiệm — Aerosonde FC hybrid, bay vòng kín đến khi cạn nhiên liệu (E3)

> **Bản chốt 03/11/2026.** Bản đầy đủ có công thức nằm ở `bao_cao/thiet_ke_E3.pdf` (ngoài repo).
> Code đã cài theo bản chốt. Các chi tiết cài đặt mà PDF không nêu cụ thể nằm ở mục 8.

**Nguyên tắc:**
- Mọi lựa chọn khó hoặc chưa có số liệu đều được **quét lưới** hoặc ghi rõ là giả thiết (GT).
- Số liệu có nguồn ghi mã [Rxx] (tài liệu đã đọc toàn văn).
- Dải tham số máy bay suy ra từ **đặc tính bay tính bằng chính mô hình** (`checks/flight_envelope.m`).
- **Giới hạn cứng không bao giờ bị vượt** (mục 2.1).

## 1. Nền máy bay: Aerosonde [R37, App. E]

**Nguồn và độ tin cậy:**
- Tham số lấy từ Beard & McLain [R37, Bảng E.1–E.2]. [R37] cho biết bộ tham số gốc từ AeroSim Blockset (Unmanned Dynamics).
- Aerosonde thật: sải 2.9 m, rỗng khoảng 8 kg, MTOW 13–14 kg, động cơ xăng khoảng 1 kW. Đây là nguồn bách khoa, chỉ dùng để kiểm tra kích cỡ.

**Kiểm tra hợp lý** (mô hình, bus 42.9 V, CG tham chiếu):

| Đại lượng | Giá trị | Nhận xét |
|---|---|---|
| CL_max | 1.23 (α0 = 14°) | α0 = 27° của [R37] cho CL_max = 2.42, phi thực tế → GT α0 = 14° |
| Vận tốc stall | 16.2 / 17.6 / 18.9 / 20.1 m/s ở 11 / 13 / 15 / 17 kg | — |
| Vận tốc trim khả thi | 18–31 m/s (11 kg) … 22–30 m/s (17 kg) | → lưới V_a 22–28 m/s |
| Công suất điện ở 25 m/s | 394–465 W | → FC 600 W |
| Góc leo tối đa ở 25 m/s | 6.5–11.4° | → lưới γ̄ 2–10° |
| Bank quay vòng đều tối đa | 40–57° | → lưới φ̄ 10–50° |
| Dự trữ ổn định ở CG tham chiếu | 0.49 c̄ (rất cao) | → CG là biến thiết kế, dịch lùi 2–8 cm |
| Các mode | short-period −4.7 ± 9.7i, phugoid −0.04 ± 0.50i, roll −21, dutch-roll −1.4 ± 5.8i, spiral +0.05 | hợp lý |

**Giả thiết (GT):**
- Servo: [R37] không cho thông số. Dùng bậc 2, ωn = 100 rad/s, ζ = 0.707, trễ 0.07 s, tốc độ 200°/s, giới hạn góc 25°.
- Hệ số khí động: bất định 10% (1σ), vì [R37] không cho độ lệch chuẩn. Mỗi kịch bản lấy một mẫu (`ac_perturb`).
- Vận tốc nào không trim được (elevator cần vượt 25°) thì bị loại khỏi các kiểu bay của chuyến bay đó.

## 2. Hệ nguồn

- **Kiến trúc:** bán chủ động [R3]: FC và siêu tụ qua DC/DC (hiệu suất 0.95, GT), pin nối thẳng bus.
- **FC:** đường phân cực AeroStack [R24], co theo diện tích cell; trễ kích hoạt τ = Td/3 [R19]; giới hạn tốc độ tăng dòng 2 A/s cho stack 200 W [R25].
  - Hai cấu hình:
    - **nhẹ:** 600 W @ 458 W/kg [R3], bình 1 h, khoảng 13.7 kg;
    - **nặng:** 600 W @ 224 W/kg [R5], bình 2 h, khoảng 16.8 kg.
  - Td = 10 s (danh định). S1 xét Td = 1 và 20 s.
- **Bình H₂:** composite 1.4% H₂ theo khối lượng [R26]. Dung tích tính cho thời gian thiết kế ở 60% P_fc, hiệu suất 45% (GT).
  - **Mức nạp ban đầu** {25, 50, 100}% dung tích; khối lượng bình giữ nguyên.
- **Pin:** LiFePO₄ 13S (42.9 V) [R4]; lão hóa [R18].
- **Siêu tụ:** Maxwell 58 F 16 V [R3], mô hình C–ESR. A7 bỏ siêu tụ.
- **EMS lọc tần số** [R3], **tần số cắt cố định f_c = 0.1 Hz** (GT): FC nhận thành phần tần số thấp, siêu tụ nhận phần cao, pin bù phần còn lại. S6 xét f_c = 0.06 và 0.3 Hz.
  - Baseline EMS: lọc thích nghi theo SoC siêu tụ [R3], luật [R27], ECMS [R30].

### 2.1 Giới hạn cứng

Áp ở mọi bước tính. Lệnh có thể đòi vượt, nhưng bộ phận thật không bao giờ vượt.

| Bộ phận | Giới hạn | Tính vào m? |
|---|---|---|
| Cánh lái | góc ±25°, tốc độ ±200°/s, trễ 0.07 s (GT) | Có |
| Ga | 0–100% | Qua bão hòa động cơ |
| Động cơ | V_in = min(δt·U_bus, 44.4 V) [R37] | Có (bão hòa liên tục > 1 s) |
| Lệnh autopilot | nghiêng ≤ φ̄; góc chúc trong giới hạn (GT) | Không |
| FC | dòng [I_min, I_max]; tăng dòng ≤ 2 A/s/200 W [R25]; công suất ≤ 600 W | Không (pin, siêu tụ bù) |
| Pin | xả ≤ 10C, sạc ≤ 1C (GT); ngắt sạc khi SoC = 100% | Có (dòng cần / 10C) |
| Siêu tụ | năng lượng 25–100% [R3]; dòng ≤ 100 A (GT) | Không (pin bù) |

Khi pin chạm giới hạn xả, động cơ chỉ nhận phần công suất còn lại.

## 3. Gió (không cần địa điểm; "mùa" = mức gió)

- **Mức gió** W̄ ∈ {0, 4, 8, 12} m/s, tức 0–50% vận tốc hành trình (MAV thường gặp gió 20–50% vận tốc [R37, mục 2.4]).
- **Gió trung bình theo nhiệm vụ** (GT):
  - Lấy trong U[W̄ − 2, W̄ + 2] (không âm). Hướng trôi U[−30°, 30°] giữa hai nhiệm vụ.
  - Chuyển dần với hằng số thời gian 20 s.
- **Nhiễu Dryden tầng thấp** [R12]: hệ số cường độ U[0.5, 3] (GT); nhân đôi trong nhiệm vụ "giữ quỹ đạo trong gió giật".
- Không dùng dữ liệu gió thật (ERA5), nên **không tuyên bố "gió thật"** khi viết bài.
- Planner đọc chính xác gió hiện tại (giả thiết "ước lượng gió hoàn hảo").

## 4. Nhiệm vụ, planner, mức vi phạm

**Chuỗi nhiệm vụ** (`flight/task_specs`, `task_make`): ngẫu nhiên theo seed, giống nhau với mọi chính sách. **Không có hạn hoàn thành.**

| Loại | Hình học (GT) |
|---|---|
| Chuyển chặng | waypoint 1–2.5 km, lệch hướng ±120° |
| Đổi hướng | 90/135/180° rồi bay thẳng 400 m |
| Leo/hạ | 20–60 m (giữ trong 60–250 m) |
| Bay vòng chờ | 2 vòng, bán kính 120–250 m |
| Quét luống cày | 4 đường dài 400–700 m, cách nhau 80–160 m |
| Giữ quỹ đạo trong gió giật | thẳng 1.5 km, nhiễu ×2 |

- **Nhiệm vụ đạt:** bay xong với m ≤ 1.
- **Giới hạn chống kẹt:** quá max(3 T_dn, T_dn + 120 s) thì bỏ nhiệm vụ. T_dn = độ dài đường bay / 25 m/s, cung quay tính với bank 30°, leo/hạ ở 6° (kiểu bay danh nghĩa).
- **Bám đường** theo trường vector [R37, Eq. 10.9, 10.15], roll feed-forward [R37, Eq. 10.18], chuyển chặng bằng nửa mặt phẳng [R37, ch. 11].
- **Autopilot:** successive loop closure [R37, ch. 6] cộng yaw damper, chống tích phân dồn. Gain thiết kế tại 7 vận tốc 22–28 m/s.

**Planner** (`flight/planner_decide`), MPC cuốn chiếu trên tập rời rạc:
- **Khi nào quyết định:** mỗi 2 s và đầu mỗi nhiệm vụ; chân trời 10 s [R37, ch. 12].
- **Kiểu bay:** lưới 315 = φ̄ {10, 15, …, 50}° × γ̄ {2, 4, 6, 8, 10}° × V_a {22, …, 28} m/s. Danh nghĩa (30°, 6°, 25 m/s).
- **Hàm chi phí** (g H₂): J = J_nl + J_tg + J_đk + J_rr.
  - J_nl = ΔM − f_ht·d; ΔM = H·(ṁ_H2 + s_eq·P_b/LHV), s_eq dạng ECMS [R30], s0 = 2.5, k = 3 (GT).
  - J_tg = κ_t·ṁ_ht·(H − d/V_ref).
  - J_đk = κ_u·ṁ_ht·H·ū², ū² = mức dùng cánh lái (góc và tốc độ, chuẩn hóa theo giới hạn), dạng phạt bậc hai như [R11].
  - J_rr = p_vp·f_ht·L_nv (H₂ để bay lại nhiệm vụ); bằng 0 nếu nhiệm vụ đã có vi phạm.
  - ṁ_ht, f_ht: suất tiêu hao H₂ hành trình ở 25 m/s, tính trước chuyến bay theo cấu hình. κ_t = κ_u = 1 (GT).
- **Ràng buộc:**
  - R1: cận trên conformal của m (AFP, ε = 0.1) ≤ 1;
  - R2: cận trên conformal 99% của α_p99/α0 ≤ 1 (δ = 0.01);
  - R3: SoC dự báo cuối chân trời ≥ 35% (dự trữ 30% [R27] + biên 5%, GT);
  - R4: vận tốc trim được.
  - Không kiểu bay nào đạt: bỏ R1; nếu vẫn không có, chọn a₀.₉₉ nhỏ nhất.

**Mức vi phạm** của một cửa sổ hay một nhiệm vụ:
- m = max(lệnh lái / 25°, α_p99 / α_lim, bão hòa tốc độ lái liên tục / 0.2 s, bão hòa động cơ liên tục / 1 s, dòng pin cần / 10C).
- Vi phạm khi m > 1; rơi thì m = 2. α_lim = α0 − 2° = 12°. Ngưỡng 0.2 s và 1 s là GT.

**Điều kiện dừng chuyến bay:** hết H₂; SoC < 30% (dự trữ hạ cánh [R27]); rơi (h < 5 m); hoặc quá 4 h.

**Bước tích phân** 0.01 s (RK4).
- Kiểm tra ở gió lặng: so với 0.0025 s, H₂, SoC và thời gian nhiệm vụ lệch dưới 0.5%, và phân loại vi phạm giống nhau.
- Ở gió có nhiễu, chuỗi nhiễu phụ thuộc bước tích phân, nên chỉ so được ở mức thống kê.

## 5. Thí nghiệm

| | E3X: thăm dò (huấn luyện) | E3: đánh giá |
|---|---|---|
| Số chuyến bay | 2000 × 10 phút | 22 chính sách × 144 kịch bản + 12 bản độ nhạy × 48 = 3744 |
| Chính sách | kiểu bay ngẫu nhiên liên tục (φ̄ U[10, 50]°, γ̄ U[2, 10]°, V_a U[22, 28] m/s), giữ 10 s | xem mục 6 |
| Cấu hình | nhẹ/nặng; CG U[−8, 0] cm; Td {1, 10, 20} s; f_c {0.06, 0.1, 0.3} Hz; 20% không siêu tụ | nhẹ/nặng × CG {−6, −2} cm; Td 10 s; f_c 0.1 Hz |
| Gió | W̄ U[0, 12] | W̄ {0, 4, 8, 12} |
| Nhiên liệu, trạng thái đầu | SoC U[0.35, 0.95], siêu tụ U[0.3, 1] | mức nạp {25, 50, 100}%; SoC U[0.5, 0.9], siêu tụ U[0.3, 1] |
| Seed | mỗi chuyến một seed | 3 (cùng nhiệm vụ, gió, khí động cho mọi chính sách) |

**Huấn luyện** (`ai/e3_train_all`):
- **Dữ liệu:** mỗi cửa sổ 10 s của E3X là một mẫu, 32 đặc trưng (`flight/e3_features`).
- **Chia dữ liệu:** theo chuyến bay, 60/20/20 cho huấn luyện / hiệu chuẩn / kiểm tra.
- **AFP:** rừng hồi quy lượng tử [R39] τ = 0.9 cộng CQR một phía [R32], ε = 0.1, hiệu chuẩn theo nhóm mức gió; p_vp = P(m > 1) đọc từ cùng rừng.
- **Rừng biên stall:** phân vị 0.99 của α_p99/α0 cộng CQR, δ = 0.01.
- **Mô hình thay thế:** quãng đường tiến, H₂, công suất pin + siêu tụ, mức dùng cánh lái ū².
- **Learning curve** (`ai/e3_learning_curve`): 25 → 1200 chuyến bay, 3 tập con ngẫu nhiên mỗi mốc, checkpoint mô hình ở mỗi mốc; pinball loss [R39, Eq. 3], độ phủ, recall, báo động giả, AUC; lỗi OOB theo số cây.
- **Báo cáo AI rồi dừng chờ duyệt** trước khi chạy E3.

**Thước đo** (`ai/e3_analysis`):
- **Chính:** số nhiệm vụ đạt trước khi cạn nhiên liệu.
- **Phụ:**
  - tỉ lệ rơi, số vi phạm, thời gian chạm từng giới hạn cứng;
  - H₂ tương đương (đã hiệu chỉnh SoC) trên mỗi nhiệm vụ đạt;
  - thời gian mỗi nhiệm vụ, mức dùng cánh lái trung bình;
  - chi phí vận hành USD trên nhiệm vụ đạt (giá [R14], suy giảm FC [R16][R17], pin [R18]).
- **Độ phủ thực tế trên quỹ đạo của chính sách:** sau mỗi quyết định, đo m và α_p99 trên 10 s tiếp theo, so với cận trên R1, R2 của kiểu bay đã chọn.
- **So sánh:** theo cặp với ours trên cùng kịch bản, kiểm định Wilcoxon.

## 6. Chính sách, ablation, độ nhạy

| Mã | Mô tả |
|---|---|
| ours | J (mục 4), R1–R4 và thứ tự nới, mỗi 2 s |
| fixed_gentle / nominal / aggr | Kiểu bay cố định (15°, 2°) / (30°, 6°) / (45°, 10°) ở 25 m/s |
| nominal_adapt / rule / ecms | Kiểu bay danh nghĩa; EMS lọc thích nghi [R3] / luật [R27] / ECMS [R30] |
| min_risk | Cùng R1–R4 và thứ tự nới như ours, chọn cận trên R1 nhỏ nhất thay cho J nhỏ nhất |
| A1 | Bỏ mô hình vi phạm m: không R1, J_rr = 0; giữ R2 |
| A2 | Không hiệu chỉnh conformal (R1, R2 dùng phân vị thô) |
| A3 | Conformal chung cho R1, R2, không chia nhóm gió |
| A4 / A5 / A6 | Không đặc trưng cấu hình / gió / trạng thái điện (huấn luyện lại); A5 dùng conformal chung |
| A7 | Bỏ siêu tụ (phần cứng) |
| C1 | Bỏ J_rr, giữ R1 |
| C2 | Bỏ R2; khi nới thì chọn cận trên R1 nhỏ nhất |
| C3 / C4 | κ_t = 0 / κ_u = 0 |
| C5 | Cùng 4 khoản, mỗi khoản chia trung vị trên dữ liệu huấn luyện, cộng trọng số bằng nhau; trung vị J_rr chỉ lấy trên cửa sổ nhiệm vụ chưa vi phạm (ở cửa sổ khác J_rr = 0) |
| C6 | Bỏ R1, giữ J_rr |
| ours_deg | Thêm chi phí suy giảm quy ra H₂ theo giá [R14] |
| S1 … S6 (48 kịch bản) | Td 1/20 s; chu kỳ 1/4 s; κ_t 0.5/2; κ_u 0.5/2; δ 0.005/0.02; f_c 0.06/0.3 Hz |

## 7. Hạ tầng

- Repo public; workflow `.github/workflows/e3.yml` chạy tối đa 20 job song song.
  - `stage=explore`: 20 job E3X, gộp thành `E3X-all`.
  - Huấn luyện và báo cáo AI từ dữ liệu E3X, rồi **dừng chờ duyệt**.
  - Đánh giá E3 dùng lại bộ mô hình đã duyệt: 120 job (GitHub dừng job chạy quá 6 giờ), gộp và phân tích.
  - `stage=test`: vài chuyến bay ngắn để kiểm tra quy trình.
- **Checkpoint:**
  - `ckpt_<job>.mat` ghi sau mỗi chuyến bay;
  - `fckpt_<job>.mat` ghi trạng thái chuyến đang bay mỗi 5 phút tính;
  - cả hai luôn được tải lên. Khi chạy lại job lỗi, job tự lấy checkpoint của mình và chạy tiếp.

## 8. Chi tiết cài đặt (PDF không nêu cụ thể)

Các chi tiết dưới đây không đổi ý tưởng của bản chốt; chúng là lựa chọn cần thiết để cài đặt, ghi lại để kiểm tra.

| Chỗ | Cách làm | Lý do |
|---|---|---|
| R4 (vận tốc trim được) | Trim hội tụ, lái trong ±25°, ga trong [0, 1] và α ≤ α_lim | "Giữ được bay bằng" phải nằm trong giới hạn cứng |
| Bay thăm dò | V_a bốc liên tục trong khoảng trim được liền mạch quanh 25 m/s | Không bay ở vận tốc không trim được |
| Trần công suất động cơ | Khi pin chạm 10C, điện áp động cơ bị giới hạn để công suất ≤ công suất còn lại của FC, siêu tụ và pin ở giới hạn (lấy từ bước trước, trễ 0.01 s) | "Động cơ chỉ nhận phần công suất còn lại" |
| Thành phần (5) của m | Dòng pin cần tính theo nhu cầu động cơ khi chưa bị giới hạn | Nếu tính sau khi giới hạn thì tỉ số không bao giờ vượt 1 |
| Thành phần (4) của m | Bão hòa khi lệnh ga (sau trễ) bằng 1 hoặc điện áp lệnh ≥ điện áp cấp được | Đúng định nghĩa "điện áp tối đa cấp được" |
| Nạp pin vượt giới hạn hoặc pin đầy | Giảm FC (giảm dòng tự do); nếu FC đã ở dòng tối thiểu mà vẫn dư thì xả trên điện trở (GT), ghi thời gian và năng lượng | Để giới hạn nạp không bao giờ bị vượt |
| Hiệu chỉnh R2 theo nhóm gió | Chỉ khi mọi nhóm có n_nhóm·δ ≥ 20 mẫu vượt kỳ vọng (GT); không thì hiệu chỉnh chung | PDF: "nếu quá ít thì hiệu chỉnh chung" |
| Ngưỡng chạy không tải / tải cao của FC (chỉ dùng để đánh giá suy giảm) | 20% / 80% P_max theo [R10] | Thay giá trị GT 5% bằng giá trị có nguồn |
| Hiệu chỉnh SoC và siêu tụ trong EMS | k_soc = 2, k_e = 0.5 (GT), tương ứng hằng số thời gian hồi phục khoảng 5 phút (SoC) và 25 s (siêu tụ) | Đặt theo ý nghĩa vật lý thay vì số tùy ý |
| ECMS (baseline) | Đường cong FC giới hạn ở công suất định mức; siêu tụ tách tần số với cùng f_c = 0.1 Hz | Cùng giới hạn cứng và cùng cách dùng siêu tụ, chỉ khác cách chia FC/pin |
| T_max = 4 h | Giới hạn kỹ thuật, dài hơn chuyến bay dài nhất có thể; báo cáo số lần chạm (kỳ vọng 0) | Chống chạy mãi nếu có lỗi |
