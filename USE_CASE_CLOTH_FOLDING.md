# Đặc Tả Use Case: Gấp Quần Áo Tự Động (Autonomous Cloth Folding)
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Hệ thống phần cứng**: Robot OpenArm (8-DOF: 7 khớp xoay + 1 motor kẹp Gripper)  
**Cấu hình cảm biến**: **Duy nhất 01 Camera RGB trước ngực (Chest Camera)**

---

## 1. TỔNG QUAN USE CASE

* **Tên tác vụ**: Autonomous Tabletop Cloth Folding (Gấp khăn / Áo thun tự động trên mặt bàn).
* **Mục tiêu**: Huấn luyện mô hình ACT (Action Chunking with Transformers) điều khiển cánh tay robot OpenArm thực hiện thao tác kẹp mép vải, nâng theo quỹ đạo vòng cung và lật gập nếp vải phẳng phiu hoàn toàn tự động chỉ dựa vào **01 luồng camera duy nhất gắn trước ngực**.
* **Đặc thù thách thức (Deformable Object Manipulation)**:
  * Khác với vật thể rắn (Rigid body) như khối hộp hay chai nước, quần áo là **vật thể mềm (Deformable Object)** với bậc tự do biến dạng vô hạn, dễ nhăn nhúm, trượt nếp và tự che khuất (self-occlusion).
  * Đòi hỏi thao tác khéo léo (Fine Manipulation): Phải tiếp cận chính xác mép/góc vải, kẹp nhíp (Pinch grasp) vừa đủ lực, nâng và lật theo biên độ cong mềm mại để không làm bung nếp gấp.

---

## 2. THIẾT LẬP MÔI TRƯỜNG & TẦM NHÌN (ENVIRONMENT SETUP)

| Thành phần | Đặc tả kỹ thuật | Ghi chú vận hành |
| :--- | :--- | :--- |
| **Vị trí Camera ngực** | Gắn cố định chính diện ngực robot, chúc góc xuống bàn $\approx 40^\circ - 45^\circ$ | Bao quát toàn bộ mặt bàn thao tác và toàn bộ tấm vải trải rộng |
| **Độ phân giải & Tần số** | RGB $640 \times 480$ pixels @ 30 FPS hoặc 60 FPS | Góc mở FOV $\ge 80^\circ$ để thấy cả 4 góc của áo/khăn và cánh tay |
| **Mặt bàn thao tác** | Mặt bàn $80 \times 80\text{ cm}$, phủ thảm nỉ hoặc cao su silicone chống trượt | Giúp cố định phần thân áo/khăn, tránh việc kéo mép làm trượt cả chiếc áo |
| **Vật thể thao tác** | Khăn bông thể thao ($40 \times 40\text{ cm}$) hoặc Áo thun trẻ em/cỡ nhỏ ($40 \times 50\text{ cm}$) | Chọn màu tương phản cao với mặt bàn (ví dụ: áo trắng/xanh trên thảm đen) |
| **Đầu kẹp (Gripper)** | Đầu kẹp 2 ngón bọc cao su ma sát cao hoặc dán mút silicon mỏng | Tăng độ bám dính khi kẹp mép vải, chống tuột vải khi nâng cao |

```text
               [ CAMERA TRƯỚC NGỰC (40° - 45°) ]
                                │
                                │ (Quan sát toàn bộ nếp nhăn & mép vải)
                                ▼
       ┌──────────────────────────────────────────────┐
       │                                              │
       │       [ Mép góc trái ] ──> [ Mép góc phải ]  │
       │       ┌─────────────────────────────┐        │
       │       │                             │        │
       │       │    ÁO THUN / KHĂN VẢI       │        │
       │       │                             │        │
       │       └─────────────────────────────┘        │
       │            MẶT BÀN CHỐNG TRƯỢT               │
       └──────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Ưu thế của Camera Ngực đối với bài toán Gấp Quần Áo**:
> Trong bài toán gấp vải, tấm vải trải rộng trên mặt bàn nên camera trước ngực có tầm nhìn từ trên cao xuống (Overhead-angled view) cực kỳ lý tưởng. Quỹ đạo lật nếp gấp di chuyển theo phương ngang/vòng cung từ mép này sang mép kia, giúp **hạn chế tối đa hiện tượng cánh tay che khuất vùng nếp gấp**, tốt hơn nhiều so với thao tác gắp vật thể nhỏ.

---

## 3. QUY TRÌNH THAO TÁC 6 GIAI ĐOẠN (TIMELINE CHI TIẾT)

```text
[1. Standby / Perception] ──> [2. Approach Corner] ──> [3. Pinch Grasp]
           │                                                  │
           ▼                                                  ▼
[6. Retract & Reset]   <── [5. Release & Flatten]  <── [4. Arc-Fold Trajectory]
```

1. **Giai đoạn 1 - Standby / Perception (Quan sát & Đánh giá trạng thái vải)**:
   * Cánh tay ở vị trí Home (co gọn phía trên).
   * Camera ngực chụp ảnh chiếc áo/khăn trải trên bàn, mô hình ACT nhận diện vị trí góc vải và mép vải cần gập.
2. **Giai đoạn 2 - Approach Corner (Tiếp cận góc vải)**:
   * Cánh tay hạ từ từ từ trên xuống, đầu kẹp mở góc $30^\circ - 45^\circ$, tiếp cận mép biên góc vải cách mặt bàn $\approx 1 - 2\text{ cm}$.
3. **Giai đoạn 3 - Pinch Grasp (Kẹp mép vải)**:
   * Hạ đầu ngón kẹp chạm mép vải và kích hoạt motor Joint 8 (Gripper) đóng lại với lực vừa đủ ($I \approx 1.0 - 1.5\text{ A}$).
   * Độ mềm dẻo (Compliance): Khớp cổ tay có độ nhún nhẹ ($K_p \approx 12 - 15$) để đầu kẹp ép sát mặt bàn gắp trúng lớp vải mà không làm kẹt motor hay trầy mặt bàn.
4. **Giai đoạn 4 - Arc-Fold Trajectory (Lật nếp gấp theo quỹ đạo vòng cung)**:
   * Nâng mép vải lên cao $\approx 10 - 15\text{ cm}$ theo đường cong Parabol mượt mà hướng sang cạnh đối diện.
   * Chuyển động liên tục, không dừng đột ngột để trọng lực giữ phẳng nếp vải tự nhiên.
5. **Giai đoạn 5 - Release & Flatten (Nhả kẹp & Vuốt phẳng)**:
   * Hạ mép vải tiếp xúc chính xác lên nửa thân áo còn lại.
   * Mở kẹp Gripper nhả vải. Đầu kẹp miết nhẹ ngang một hành trình ngắn $3 - 5\text{ cm}$ tạo nếp gấp phẳng phiu.
6. **Giai đoạn 6 - Retract & Reset (Thu tay về vị trí chờ)**:
   * Nhấc cánh tay thẳng đứng lên khỏi mặt bàn $15\text{ cm}$ để không làm xô lệch nếp vừa gấp, sau đó thu về tư thế Standby sẵn sàng cho chu kỳ tiếp theo.

---

## 4. ĐIỀU CHỈNH THÔNG SỐ ĐIỀU KHIỂN KHỚP (IMPEDANCE & GAINS TUNING)

Do đặc thù kẹp vải sát mặt bàn, bộ điều khiển Damiao MIT Mode cần tinh chỉnh độ cứng (Stiffness) mềm mại hơn so với việc gắp vật cứng:

| Nhóm khớp | Động cơ Damiao | Độ cứng vị trí ($K_p$) | Giảm chấn vận tốc ($K_d$) | Mục đích kỹ thuật |
| :--- | :---: | :---: | :---: | :--- |
| **Khớp vai & gốc (J1, J2, J3)** | DM8009 & DM4340 | $25.0 - 30.0$ | $1.2 - 1.5$ | Giữ vững khung nâng tải trọng cánh tay khi vươn xa |
| **Khớp khuỷu & cổ tay (J4, J5, J6, J7)** | DM4310 | **$12.0 - 15.0$** | **$0.8 - 1.0$** | **Độ mềm dẻo cao (Compliance)**: Chống quá dòng khi chạm mặt bàn |
| **Motor kẹp Gripper (J8)** | DM4310 | $10.0$ | $0.5$ | Kẹp nhíp nhạy, giới hạn lực dòng điện $I_{max} = 1.5\text{ A}$ |

---

## 5. TIÊU CHÍ ĐÁNH GIÁ THÀNH CÔNG (KPIS)

* **Tỷ lệ thành công (Success Rate)**: $\ge 80\%$ trong 20 lần thử nghiệm với các độ lệch ban đầu của áo/khăn ($\pm 5\text{ cm}$, xoay góc $\pm 15^\circ$).
* **Độ chính xác nếp gấp (Folding Alignment Error)**: Mép gấp lệch so với mép đối diện $\le 3\text{ cm}$.
* **Thời gian hoàn thành (Cycle Time)**: $12 - 16\text{ giây}$ cho một chu kỳ gấp 1 nếp hoàn chỉnh.
* **Tiêu chí tính là thất bại**:
  * Kẹp hụt mép vải (kẹp đóng nhưng không có vải bên trong).
  * Vải bị tuột giữa đường khi đang nâng theo quỹ đạo vòng cung.
  * Nếp gấp bị nhăn nhúm nặng hoặc mép lệch $> 5\text{ cm}$.
  * Đầu kẹp tì quá mạnh làm motor báo lỗi quá dòng (Flashing RED LED).

---

## 6. HƯỚNG DẪN THU THẬP DỮ LIỆU MẪU (TELEOPERATION CHO BÀI TOÁN GẤP ÁO)

Khi team thu thập dữ liệu sử dụng tay cầm (Gamepad/Controller) hoặc tay dẫn đường (Leader Arm) để ghi demo 50 episodes:

1. **Số lượng Episodes**: Thu thập tối thiểu **50 episodes** chất lượng cao:
   * *30 episodes*: Khăn/áo đặt ngay ngắn ở vị trí chuẩn trung tâm.
   * *10 episodes*: Áo đặt lệch sang trái/phải $\pm 3 - 5\text{ cm}$ hoặc xoay góc $\pm 10^\circ$.
   * *10 episodes*: Áo có nếp nhăn nhẹ ban đầu để mô hình học tính thích nghi.
2. **Quy tắc quỹ đạo**:
   * Khi nhấc mép vải lên, luôn kéo căng vừa phải theo quỹ đạo vòm cung mượt mà, không giật mạnh.
   * Thời lượng mỗi episode ghi hình: $\approx 12 - 15\text{ giây}$ (tương đương $600 - 750$ timesteps ở tần số 50Hz).
