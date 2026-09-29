# BỘ QUY TẮC PHÁT TRIỂN DỰ ÁN OPENARM CAN (WORKSPACE RULES)

> **BẮT BUỘC ĐỐI VỚI MỌI AI AGENT VÀ DEVELOPER KHI LÀM VIỆC TRÊN REPOSITORY NÀY**  
> Mọi đoạn code được sinh ra, chỉnh sửa hoặc refactor đều phải tuân thủ nghiêm ngặt theo tài liệu kiến trúc [ARCHITECTURE.md](file:///home/quan/openarm_can/ARCHITECTURE.md). Tuyệt đối không được vi phạm các bất biến kiến trúc (Architectural Invariants) dưới đây.

---

## 1. NGUYÊN TẮC CỐT LÕI (ARCHITECTURAL INVARIANTS)

1. **BACKEND LÀ HUB ĐIỀU PHỐI DUY NHẤT (NO BYPASS):**
   - **CẤM:** Tuyệt đối không cho phép Model AI, Teleop, hoặc UI gửi frame CAN trực tiếp xuống phần cứng robot.
   - **BẮT BUỘC:** Toàn bộ lệnh điều khiển từ Model AI (`/joint_state_cmd` 50 Hz) và Teleop (`/joint_state_cmd` 50–100 Hz) phải đi qua `Backend Middleware`.
   - Backend chịu trách nhiệm duy nhất về giao tiếp hai chiều với Robot qua bus CAN (400 Hz).

2. **TRIỆT TIÊU ĐỘ LỆCH PHA TÍN HIỆU (ZERO-PHASE-SHIFT DATA INGESTION):**
   - **CẤM:** Model AI không được tự ý subscribe các topic raw rời rạc (`/camera` 25 Hz và `/joint_states` 100 Hz) trực tiếp khi huấn luyện hay suy luận vì sẽ gây lệch pha tín hiệu (phase shift / timestamp jitter).
   - **BẮT BUỘC:** Model AI chỉ nhận dữ liệu quan sát đã qua gióng hàng thời gian từ `Data Recorder` (tập tin HDF5/Zarr khi huấn luyện, hoặc IPC Shared Memory Buffer khi suy luận thời gian thực).

3. **AN TOÀN LÀ TIÊN QUYẾT (SAFETY GUARD & SPLINE INTERPOLATION):**
   - Mọi lệnh điều khiển góc khớp trước khi nạp xuống CAN Node của Robot bắt buộc phải trải qua:
     1. **Safety Check:** Kiểm tra giới hạn góc khớp, giới hạn vận tốc, gia tốc, và kiểm tra timeout mất kết nối (heartbeat > 50 ms).
     2. **Spline Interpolation:** Nội suy mượt (Cubic / Quintic Spline) từ tần số lệnh (50 Hz hoặc 50–100 Hz) lên tần số chuẩn **400 Hz** của CAN bus.
   - **E-Stop (Dừng khẩn cấp):** Có độ ưu tiên cao nhất (Priority 0). Bất cứ khi nào nhận tín hiệu E-Stop từ UX/UI hoặc Safety Guard, Backend phải lập tức vô hiệu hóa phát lệnh động cơ trong chu kỳ 2.5 ms tiếp theo.

---

## 2. KHẾ ƯỚC TẦN SỐ VÀ CHU KỲ (TIMING & FREQUENCY CONTRACTS)

Khi viết bất kỳ worker, thread, timer hay vòng lặp nào, Agent phải tuân thủ đúng tần số chuẩn:

| Tên Module / Luồng | Tần số bắt buộc | Chu kỳ (\( \Delta t \)) | Ghi chú kỹ thuật |
| :--- | :--- | :--- | :--- |
| **Robot CAN Bus (Feedback & Cmd)** | **400 Hz** | **2.5 ms** | Real-time loop, yêu cầu timing jitter < 0.2 ms |
| **Camera RGB-D Ingestion** | **25 Hz** | **40.0 ms** | Đồng bộ Color + Depth frame |
| **`/joint_states` Publisher** | **100 Hz** | **10.0 ms** | Lọc từ CAN 400Hz, cấp cho Teleop, UI, Recorder |
| **`/camera` Stream Publisher** | **25 Hz** | **40.0 ms** | Cấp cho UI và Recorder |
| **Teleop `/joint_state_cmd`** | **50–100 Hz**| **10.0–20.0 ms** | Lệnh tay điều khiển từ xa |
| **Data Recorder Alignment Loop** | **50 Hz** | **20.0–25.0 ms** | Gióng hàng thời gian đa cảm biến, xuất gói 50 Hz |
| **Model AI Policy Inference** | **50 Hz** | **20.0 ms** | Suy luận **1+49 action chunking** |

---

## 3. QUY ĐỊNH CHI TIẾT THEO TỪNG MODULE MÃ NGUỒN

### 3.1. `backend/`
- Thư mục `backend/can_node/` chỉ xử lý socket CAN và giao thức truyền thông phần cứng 400 Hz.
- Thư mục `backend/safety/` phải độc lập, có unit test đầy đủ cho các trường hợp: quá giới hạn góc khớp, vận tốc bất thường, timeout, và kích hoạt E-Stop.
- Thư mục `backend/interpolator/` phải đảm bảo tính toán liên tục bậc 2 (liên tục về vị trí, vận tốc, gia tốc) khi nâng xung từ 50 Hz lên 400 Hz.

### 3.2. `teleop/`
- Đóng vai trò Leader arm.
- Nhận `/joint_states` (100 Hz) để cập nhật trạng thái cánh tay.
- Xuất bản `/joint_state_cmd` (50–100 Hz) theo chuẩn format xác định tại `common/messages.py`.

### 3.3. `ui/`
- Gửi lệnh E-Stop trực tiếp đến Backend qua kênh IPC hoặc socket có độ trễ cực thấp.
- Gửi lệnh `START_RECORD` và `STOP_RECORD` đến Data Recorder cùng metadata (tên nhiệm vụ, người vận hành, v.v.).

### 3.4. `recorder/`
- Bộ gióng hàng thời gian (`temporal_aligner.py`) phải xử lý trôi thời gian (clock drift) và mất gói (packet loss).
- Đóng gói file HDF5 phải tương thích cấu trúc dataset chuẩn của ACT (Action Chunking with Transformers) và LeRobot.

### 3.5. `model_ai/`
- Mô hình phải hỗ trợ định dạng Action Chunking: xuất ra dự đoán \( 1 + 49 = 50 \) bước hành động liên tiếp mỗi chu kỳ suy luận 50 Hz.
- Khi triển khai online, lấy trạng thái quan sát từ Ring Buffer Shared Memory của Recorder, không khởi tạo luồng đọc riêng biệt từ camera/CAN.

---

## 4. QUY TẮC VIẾT CODE CỦA AGENT

1. **Kiểm tra kiểu dữ liệu tĩnh (Static Typing):** Toàn bộ code Python phải có Type Hints đầy đủ (`typing.Optional`, `typing.List`, `numpy.typing.NDArray`, v.v.).
2. **Không hard-code hằng số:** Mọi hằng số tần số (`CAN_FREQ_HZ = 400`, `RECORDER_FREQ_HZ = 50`, v.v.) phải được import từ [common/constants.py](file:///home/quan/openarm_can/common/constants.py).
3. **Quản lý tài nguyên & Exception Safe:** Tất cả kết nối SocketCAN, camera pipeline, và file HDF5 phải sử dụng Context Manager (`with ...`) hoặc có cơ chế giải phóng tài nguyên an toàn trong khối `finally` / signal handler (SIGINT, SIGTERM).
4. **Không làm rò rỉ độ trễ (Zero Uncontrolled Latency):** Không sử dụng `time.sleep()` tùy tiện trong các vòng lặp thời gian thực; sử dụng rate limiter chính xác dựa trên monotonic clock.
