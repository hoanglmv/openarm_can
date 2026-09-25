# Đặc Tả Use Case: Gấp Quần Áo Tự Động Hai Tay (Autonomous Bimanual Cloth Folding)
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Hệ thống phần cứng**: Robot Bimanual OpenArm (16-DOF: 2 tay x [7 khớp xoay + 1 motor kẹp Gripper])  
**Cấu hình cảm biến**: **Duy nhất 01 Camera RGB trước ngực (Chest Camera)**, kết nối CAN-FD qua SocketCAN `can0`.

---

## 1. TỔNG QUAN USE CASE

* **Tên tác vụ**: Autonomous Bimanual Tabletop Cloth Folding (Gấp khăn / Áo thun tự động 2 tay trên mặt bàn).
* **Mục tiêu**: Huấn luyện mô hình ACT (Action Chunking with Transformers) điều khiển hệ thống robot hai tay OpenArm (16 khớp) phối hợp nhịp nhàng: kẹp đồng thời 2 góc mép vải, nâng theo quỹ đạo vòng cung đồng bộ và lật gập nếp vải phẳng phiu hoàn toàn tự động chỉ dựa vào **01 luồng camera duy nhất gắn trước ngực**.
* **Đặc thù thách thức (Deformable Object Manipulation & Bimanual Coordination)**:
  * Khác với vật thể rắn (Rigid body) như khối hộp hay chai nước, quần áo là **vật thể mềm (Deformable Object)** với bậc tự do biến dạng vô hạn, dễ nhăn nhúm, trượt nếp và tự che khuất (self-occlusion).
  * **Yêu cầu phối hợp 2 tay khắt khe**: Thao tác gấp quần áo kích thước tiêu chuẩn bắt buộc phải dùng 2 tay. Cả 2 tay phải tiếp cận đồng thời 2 góc vải, kẹp nhíp (Pinch grasp) vừa đủ lực, nâng và lật theo biên độ cong đồng pha để nếp vải trải đều, không bị xoắn xéo hay co rúm.

---

## 2. THIẾT LẬP MÔI TRƯỜNG & TẦM NHÌN (ENVIRONMENT SETUP)

| Thành phần | Đặc tả kỹ thuật | Ghi chú vận hành |
| :--- | :--- | :--- |
| **Vị trí Camera ngực** | Gắn cố định chính diện ngực robot, chúc góc xuống bàn $\approx 40^\circ - 45^\circ$ | Bao quát toàn bộ mặt bàn thao tác, tấm vải trải rộng và cả 2 cánh tay |
| **Độ phân giải & Tần số** | RGB $640 \times 480$ pixels @ 30 FPS hoặc 60 FPS | Góc mở FOV $\ge 80^\circ$ để thấy cả 4 góc của áo/khăn và 2 cánh tay vươn ra |
| **Mặt bàn thao tác** | Mặt bàn $80 \times 80\text{ cm}$, phủ thảm nỉ hoặc cao su silicone chống trượt | Giúp cố định phần thân áo/khăn, tránh việc kéo mép làm trượt cả chiếc áo |
| **Vật thể thao tác** | Khăn bông thể thao ($40 \times 40\text{ cm}$) hoặc Áo thun trẻ em/cỡ nhỏ ($40 \times 50\text{ cm}$) | Chọn màu tương phản cao với mặt bàn (ví dụ: áo trắng/xanh trên thảm đen) |
| **Đầu kẹp 2 tay (Dual Grippers)** | Đầu kẹp 2 ngón bọc cao su ma sát cao hoặc dán mút silicon mỏng | Tăng độ bám dính khi kẹp mép vải, chống tuột vải khi nâng cao |

```text
               [ CAMERA TRƯỚC NGỰC (40° - 45°) ]
                                │
                                │ (Quan sát toàn cảnh 2 tay & nếp vải)
                                ▼
       ┌──────────────────────────────────────────────┐
       │  [Tay Trái]                      [Tay Phải]   │
       │       │                              │        │
       │       ▼                              ▼        │
       │   [Góc Trái] ──────────────────> [Góc Phải]  │
       │   ┌──────────────────────────────────────┐   │
       │   │                                      │   │
       │   │          ÁO THUN / KHĂN VẢI          │   │
       │   │                                      │   │
       │   └──────────────────────────────────────┘   │
       │            MẶT BÀN CHỐNG TRƯỢT               │
       └──────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Ưu thế của Camera Ngực đối với bài toán Gấp Quần Áo Hai Tay**:
> Trong bài toán gấp vải 2 tay, tấm vải trải rộng trên mặt bàn và 2 tay vươn sang 2 bên đối xứng. Camera trước ngực có tầm nhìn từ trên cao xuống (Overhead-angled view) cực kỳ lý tưởng, bao quát trọn vẹn cả 2 tay và mép vải mà **hoàn toàn không bị cánh tay che khuất vùng nếp gấp**.

---

## 3. QUY TRÌNH THAO TÁC 6 GIAI ĐOẠN (TIMELINE PHỐI HỢP HAI TAY)

```text
[1. Standby / Perception] ──> [2. Coordinated Approach] ──> [3. Dual Pinch Grasp]
           │                                                        │
           ▼                                                        ▼
[6. Retract & Reset]   <── [5. Release & Flatten]    <── [4. Synchronized Arc-Fold]
```

1. **Giai đoạn 1 - Standby / Perception (Quan sát & Đánh giá trạng thái vải)**:
   * Cả 2 cánh tay ở vị trí Home (co gọn đối xứng phía trên).
   * Camera ngực chụp ảnh chiếc áo/khăn trải trên bàn, mô hình ACT nhận diện vị trí 2 góc mép vải cần gập.
2. **Giai đoạn 2 - Coordinated Approach (Tiếp cận 2 góc đồng bộ)**:
   * Cả 2 cánh tay hạ từ từ từ trên xuống đối xứng, 2 đầu kẹp mở góc $30^\circ - 45^\circ$, tiếp cận 2 mép biên góc vải cách mặt bàn $\approx 1 - 2\text{ cm}$.
3. **Giai đoạn 3 - Dual Pinch Grasp (Kẹp giữ đồng thời 2 mép vải)**:
   * Hạ nhẹ 2 đầu ngón kẹp chạm mép vải và kích hoạt motor Joint 8 (Gripper) của cả 2 tay đóng lại với lực vừa đủ ($I \approx 1.0 - 1.5\text{ A}$).
   * Độ mềm dẻo (Compliance): Khớp cổ tay 2 bên có độ nhún nhẹ ($K_p \approx 12 - 15$) để đầu kẹp ép sát mặt bàn gắp trúng lớp vải mà không làm kẹt motor hay trầy mặt bàn.
4. **Giai đoạn 4 - Synchronized Arc-Fold (Nâng lật vòm cung đồng bộ)**:
   * Cả 2 tay nâng mép vải lên cao $\approx 10 - 15\text{ cm}$ theo đường cong Parabol mượt mà hướng sang cạnh đối diện.
   * Chuyển động đồng pha của cả 16 khớp, không dừng đột ngột để trọng lực giữ phẳng nếp vải tự nhiên.
5. **Giai đoạn 5 - Release & Flatten (Nhả kẹp & Vuốt phẳng)**:
   * Hạ mép vải tiếp xúc chính xác lên nửa thân áo còn lại.
   * Mở 2 kẹp Gripper nhả vải. Hai đầu kẹp miết nhẹ ngang một hành trình ngắn $3 - 5\text{ cm}$ tạo nếp gấp phẳng phiu.
6. **Giai đoạn 6 - Retract & Reset (Thu 2 tay về vị trí chờ)**:
   * Nhấc cả 2 cánh tay thẳng đứng lên khỏi mặt bàn $15\text{ cm}$ để không làm xô lệch nếp vừa gấp, sau đó thu về tư thế Standby sẵn sàng cho chu kỳ tiếp theo.

---

## 4. ĐIỀU CHỈNH THÔNG SỐ ĐIỀU KHIỂN 16 KHỚP (IMPEDANCE & GAINS TUNING)

Do đặc thù kẹp vải sát mặt bàn, bộ điều khiển Damiao MIT Mode cho cả 16 motor (8 Tay Trái + 8 Tay Phải) cần tinh chỉnh độ cứng (Stiffness) mềm mại hơn so với việc gắp vật cứng:

| Cánh tay | Nhóm khớp | Động cơ Damiao | CAN ID (Send/Recv) | Độ cứng $K_p$ | Giảm chấn $K_d$ | Mục đích kỹ thuật |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **Tay Trái** | Vai gốc (J1, J2, J3) | DM8009 / DM4340 | `0x01..0x03` / `0x11..0x13` | $25.0 - 30.0$ | $1.2 - 1.5$ | Giữ vững khung nâng tải trọng cánh tay trái khi vươn xa |
| **Tay Trái** | Khuỷu & Cổ tay (J4 - J7) | DM4340 / DM4310 | `0x04..0x07` / `0x14..0x17` | **$12.0 - 15.0$** | **$0.8 - 1.0$** | **Độ mềm dẻo cao (Compliance)**: Chống quá dòng khi chạm mặt bàn |
| **Tay Trái** | Kẹp Gripper L (J8) | DM4310 | `0x08` / `0x18` | $10.0$ | $0.5$ | Kẹp nhíp nhạy, giới hạn dòng điện $I_{max} = 1.5\text{ A}$ |
| **Tay Phải** | Vai gốc (J1, J2, J3) | DM8009 / DM4340 | `0x21..0x23` / `0x31..0x33` | $25.0 - 30.0$ | $1.2 - 1.5$ | Giữ vững khung nâng tải trọng cánh tay phải khi vươn xa |
| **Tay Phải** | Khuỷu & Cổ tay (J4 - J7) | DM4340 / DM4310 | `0x24..0x27` / `0x34..0x37` | **$12.0 - 15.0$** | **$0.8 - 1.0$** | **Độ mềm dẻo cao (Compliance)**: Chống quá dòng khi chạm mặt bàn |
| **Tay Phải** | Kẹp Gripper R (J8) | DM4310 | `0x28` / `0x38` | $10.0$ | $0.5$ | Kẹp nhíp nhạy, giới hạn dòng điện $I_{max} = 1.5\text{ A}$ |

---

## 5. TIÊU CHÍ ĐÁNH GIÁ THÀNH CÔNG (KPIS)

* **Tỷ lệ thành công (Success Rate)**: $\ge 80\%$ trong 20 lần thử nghiệm với các độ lệch ban đầu của áo/khăn ($\pm 5\text{ cm}$, xoay góc $\pm 15^\circ$).
* **Độ chính xác nếp gấp (Folding Alignment Error)**: Mép gấp lệch so với mép đối diện $\le 3\text{ cm}$.
* **Thời gian hoàn thành (Cycle Time)**: $12 - 16\text{ giây}$ cho một chu kỳ gấp 1 nếp hoàn chỉnh của cả 2 tay.
* **Tiêu chí tính là thất bại**:
  * Kẹp hụt mép vải (kẹp đóng nhưng không có vải bên trong).
  * Vải bị tuột giữa đường khi đang nâng theo quỹ đạo vòng cung.
  * Nếp gấp bị nhăn nhúm nặng hoặc mép lệch $> 5\text{ cm}$.
  * Đầu kẹp tì quá mạnh làm motor báo lỗi quá dòng (Flashing RED LED).

---

## 6. HƯỚNG DẪN THU THẬP DỮ LIỆU MẪU (TELEOPERATION CHO BÀI TOÁN GẤP ÁO HAI TAY)

Khi team thu thập dữ liệu sử dụng cặp tay dẫn đường (Leader Arms) để ghi demo 50 episodes:

1. **Số lượng Episodes**: Thu thập tối thiểu **50 episodes** chất lượng cao:
   * *30 episodes*: Khăn/áo đặt ngay ngắn ở vị trí chuẩn trung tâm bàn.
   * *10 episodes*: Áo đặt lệch sang trái/phải $\pm 3 - 5\text{ cm}$ hoặc xoay góc $\pm 10^\circ$.
   * *10 episodes*: Áo có nếp nhăn nhẹ ban đầu để mô hình học tính thích nghi.
2. **Quy tắc quỹ đạo**:
   * Khi nhấc 2 mép vải lên, luôn phối hợp 2 tay nâng đều, kéo căng vừa phải theo quỹ đạo vòm cung mượt mà, không giật mạnh lệch pha.
   * Thời lượng mỗi episode ghi hình: $\approx 12 - 15\text{ giây}$ (tương đương $600 - 750$ timesteps ở tần số 50Hz, 16 kênh góc khớp).
