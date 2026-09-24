# OpenArm CAN & Web Dashboard Control System

Thư viện phần mềm và hệ thống điều khiển cánh tay robot bimanual **OpenArm** (2 tay robot 7-DOF + 2 tay kẹp song song Gripper J8, tổng cộng 16 động cơ DaMiao DM4310 / DM8009...) thông qua giao thức SocketCAN CAN-FD.

Hệ thống bao gồm:
- **Thư viện C++ Core & Python SDK (`openarm_can`)**: Giao tiếp hiệu năng cao, độ trễ thấp với phần cứng động cơ.
- **Công cụ dòng lệnh chẩn đoán (`openarm-can-cli`)**: Quét bus CAN (discover), giám sát tham số thời gian thực (monitor), cấu hình tham số động cơ (RID).
- **Web Dashboard & 3D Digital Twin (`sim/server.py` + `sim/web/`)**: Giao diện điều khiển trực quan qua trình duyệt web với mô hình 3D song sinh số (Three.js), điều khiển thanh trượt từng khớp có khóa an toàn, phím tắt đóng/mở kẹp, giám sát lực - nhiệt độ - tần số CAN ở tần số cao.
- **Script điều khiển tay kẹp độc lập (`scripts/control_gripper.py`)**: Script dòng lệnh hỗ trợ test chu kỳ đóng/mở, tương tác bàn phím, tinh chỉnh lực kẹp an toàn.

---

## Mục lục

1. [Yêu cầu phần cứng](#1-yêu-cầu-phần-cứng)
2. [Cài đặt trên Ubuntu thuần (Native Linux)](#2-cài-đặt-trên-ubuntu-thuần-native-linux)
3. [Cài đặt trên Windows sử dụng WSL2](#3-cài-đặt-trên-windows-sử-dụng-wsl2)
4. [Biên dịch Thư viện C++ & Cài đặt Python SDK](#4-biên-dịch-thư-viện-c--cài-đặt-python-sdk)
5. [Hướng dẫn Sử dụng Hệ thống](#5-hướng-dẫn-sử-dụng-hệ-thống)
   - [Cách 1: Giao diện Web Dashboard & 3D Digital Twin (Khuyên dùng)](#cách-1-giao-diện-web-dashboard--3d-digital-twin-khuyên-dùng)
   - [Cách 2: Điều khiển tay kẹp Gripper bằng Script](#cách-2-điều-khiển-tay-kẹp-gripper-bằng-script)
   - [Cách 3: Sử dụng CLI chẩn đoán (`openarm-can-cli`)](#cách-3-sử-dụng-cli-chẩn-đoán-openarm-can-cli)
   - [Cách 4: Lập trình điều khiển bằng C++ & Python](#cách-4-lập-trình-điều-khiển-bằng-c--python)
6. [Bảng ánh xạ Khớp & CAN ID](#6-bảng-ánh-xạ-khớp--can-id)
7. [Xử lý sự cố thường gặp (Troubleshooting)](#7-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. Yêu cầu phần cứng

- **Cánh tay robot**: Robot OpenArm (1 tay đơn hoặc 2 tay đối xứng Bimanual).
- **Bộ chuyển đổi USB-CAN**: Thiết bị hỗ trợ **CAN-FD** trên SocketCAN Linux:
  - Khuyên dùng: **PEAK-System PCAN-USB Pro FD** (2 kênh: `can0` nối Tay Phải, `can1` nối Tay Trái).
  - Hoặc các dòng CAN-FD tương thích: Candlelight FD, CANable 2.0, USB-CAN FD adapter.
- **Nguồn điện**: Nguồn công suất 24V – 48V DC cho động cơ cánh tay.
- **Hệ điều hành máy tính điều khiển**:
  - **Ubuntu Linux** (22.04 LTS hoặc 24.04 LTS khuyên dùng).
  - Hoặc **Windows 10 / 11** thông qua **WSL2 (Windows Subsystem for Linux)**.

---

## 2. Cài đặt trên Ubuntu thuần (Native Linux)

Áp dụng cho máy tính cài đặt Ubuntu trực tiếp (Dual boot, Mini PC, PC công nghiệp IPC).

### Bước 2.1: Cài đặt các gói phụ thuộc

Mở Terminal và chạy lệnh:

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    can-utils \
    iproute2 \
    python3-dev \
    python3-pip \
    python3-venv
```

Cài đặt thư viện Python bổ trợ cho Web Dashboard:

```bash
pip install websockets
```

### Bước 2.2: Cấu hình cổng CAN-FD

Cắm thiết bị USB-CAN (ví dụ PCAN-USB Pro FD) vào cổng USB máy tính. Kiểm tra xem hệ thống đã nhận diện chưa:

```bash
lsusb
# Bạn sẽ thấy thiết bị: PEAK System PCAN-USB Pro FD
```

Kích hoạt các cổng mạng CAN với thông số CAN-FD chuẩn (Nominal: 1 Mbps, Data: 5 Mbps):

```bash
# Kích hoạt cổng can0 (Tay Phải - Right Arm)
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on
sudo ip link set can0 up

# Kích hoạt cổng can1 (Tay Trái - Left Arm)
sudo ip link set can1 type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on
sudo ip link set can1 up
```

Kiểm tra trạng thái các cổng CAN:

```bash
ip link show | grep can
# Cả can0 và can1 phải hiển thị trạng thái "UP" và "mtu 72" (CAN-FD)
```

*(Tùy chọn) Tự động kích hoạt CAN khi cắm thiết bị*: Bạn có thể sao chép file rule cấu hình hoặc dùng script tự động của thư viện:
```bash
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

---

## 3. Cài đặt trên Windows sử dụng WSL2

Áp dụng khi bạn dùng máy tính Windows 10 / 11 và chạy môi trường điều khiển trong WSL2 Ubuntu.

### Bước 3.1: Chuẩn bị WSL2 trên Windows

1. Mở PowerShell với quyền Administrator và cài đặt WSL2 (nếu chưa có):
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
2. Khởi động vào Ubuntu WSL2 và cập nhật gói:
   ```bash
   sudo apt update && sudo apt install -y build-essential cmake git ninja-build can-utils iproute2 python3-dev python3-pip python3-venv
   pip install websockets
   ```

### Bước 3.2: Cài đặt công cụ `usbipd-win` trên Windows

Để chuyển tín hiệu USB của PCAN-USB từ Windows vào WSL2, ta dùng công cụ mã nguồn mở Microsoft `usbipd-win`:

1. Trên Windows, mở **PowerShell (Administrator)** và chạy lệnh cài đặt:
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
   *(Hoặc tải file cài đặt `.msi` mới nhất từ [usbipd-win Releases](https://github.com/dorssel/usbipd-win/releases))*.

2. Khởi động lại máy tính (hoặc khởi động lại WSL bằng `wsl --shutdown`) nếu được yêu cầu.

### Bước 3.3: Gắn (Attach) thiết bị USB-CAN vào WSL2

1. Cắm USB-CAN vào cổng USB của máy tính Windows.
2. Mở **PowerShell (Administrator)** trên Windows và liệt kê thiết bị USB:
   ```powershell
   usbipd list
   ```
   Bạn sẽ thấy dòng chứa thiết bị PCAN:
   ```text
   BUSID  VID:PID    DEVICE                                   STATE
   1-3    0c72:0011  PCAN-USB Pro FD CAN, PCAN-USB Pro FD LIN  Not shared
   ```
3. Chia sẻ thiết bị (chỉ cần làm 1 lần duy nhất cho mỗi cổng cắm):
   ```powershell
   usbipd bind --busid 1-3
   ```
4. Gắn thiết bị vào máy ảo WSL:
   ```powershell
   usbipd attach --wsl --busid 1-3
   ```
   *(Thay `1-3` bằng đúng BUSID trên máy của bạn)*.

### Bước 3.4: Nạp Kernel Module và Kích hoạt CAN trên WSL2

Chuyển sang cửa sổ terminal **WSL2 (Ubuntu)**:

```bash
# 1. Nạp các module kernel CAN cần thiết
sudo modprobe vhci-hcd
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-dev
sudo modprobe peak_usb

# 2. Kiểm tra xem thiết bị đã xuất hiện chưa
lsusb
# Sẽ thấy: Bus 001 Device ... PEAK System PCAN-USB Pro FD

# 3. Kích hoạt can0 và can1 ở chế độ CAN-FD
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on
sudo ip link set can0 up

sudo ip link set can1 type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on
sudo ip link set can1 up
```

> [!TIP]
> **Kết nối tự động 1-Click ngay trên Web Dashboard:**
> Khi chạy file `python3 sim/server.py`, hệ thống đã tích hợp sẵn tính năng tự động tìm kiếm BUSID của thiết bị USB và tự gọi `usbipd attach`. Bạn cũng có thể bấm nút **🔌 Connect USB Robot** trên góc giao diện web để hệ thống tự động kết nối mà không cần gõ lệnh thủ công.

---

## 4. Biên dịch Thư viện C++ & Cài đặt Python SDK

Clone mã nguồn dự án:

```bash
git clone https://github.com/enactic/openarm_can.git
cd openarm_can
```

### 4.1. Biên dịch Thư viện C++

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -GNinja
cmake --build build
sudo cmake --install build
sudo ldconfig
```

Sau khi cài đặt, công cụ chẩn đoán `openarm-can-cli` và thư viện `libopenarm-can.so` sẽ sẵn sàng trên hệ thống.

### 4.2. Cài đặt Python SDK (`openarm_can`)

```bash
cd python
pip install .
cd ..
```

Kiểm tra cài đặt:
```bash
python3 -c "import openarm_can as oa; print('OpenArm CAN SDK Version:', oa.__version__ if hasattr(oa, '__version__') else 'Installed OK')"
```

---

## 5. Hướng dẫn Sử dụng Hệ thống

### Cách 1: Giao diện Web Dashboard & 3D Digital Twin (Khuyên dùng)

Giao diện trực quan tích hợp đầy đủ mô hình 3D cử động theo thời gian thực, điều khiển từng khớp có khóa an toàn, điều khiển thanh kẹp ngang, đồ thị đo đạc lực và nhiệt độ.

1. **Khởi chạy máy chủ Dashboard**:
   ```bash
   python3 sim/server.py
   ```
   - Server tự động phát hiện phần cứng:
     - Nếu có `can0`/`can1`: Tự động chạy ở **REAL ROBOT HARDWARE MODE**.
     - Nếu chưa cắm USB: Tự động chạy ở **SIMULATION MODE (vcan0)** để bạn chạy thử nghiệm phần mềm an toàn.
   - Khi cắm hoặc rút USB, server sẽ tự động chuyển đổi chế độ nóng (Hotplug) mà không cần tắt mở lại server.

2. **Mở trình duyệt Web**:
   Truy cập vào địa chỉ:
   ```text
   http://localhost:8888
   ```

3. **Các tính năng chính trên giao diện**:
   - **Mô hình 3D Digital Twin**: Dựng lại cánh tay robot OpenArm bằng Three.js, tự động xoay các khớp theo phản hồi thực tế từ robot.
   - **Thanh điều khiển End-Effector Grippers (J8 - Thanh kẹp ngang)**:
     - Nút **Đóng (0mm)**: Đóng kẹp hoàn toàn.
     - Nút **50% (21.5mm)**: Mở kẹp một nửa hành trình.
     - Nút **Mở (43mm)**: Mở kẹp tối đa.
     - Nút **⇄ Đảo chiều**: Đổi chiều đóng/mở của động cơ nếu lắp ngược cơ khí.
     - Thanh trượt điều chỉnh tự do từ `0.0 mm` đến `43.0 mm`.
     - Tích hợp bảo vệ chống kẹt (Stall protection): tự động phục hồi nếu kẹp chạm vào vật thể.
   - **Điều khiển Khớp J1 đến J7**:
     - Tích hợp công tắc **Khóa an toàn (Lock/Unlock)** trên từng khớp để tránh sơ ý chạm vào thanh trượt.
     - Nút chuyển tab xem riêng **Tay Trái (Left)**, **Tay Phải (Right)** hoặc **Đồng bộ cả hai tay (Dual-Arm Sync)**.
   - **Bộ giới hạn vận tốc an toàn (Speed Profile)**:
     - Giới hạn vận tốc mặc định `0.25 rad/s` (~14°/s) giúp cánh tay chuyển động mượt mà, chống rung lắc cơ khí.
   - **Đồng bộ trạng thái ban đầu (Sync Robot State)**:
     - Bấm nút **Sync Robot State** để đọc vị trí thực tế của robot trước khi kích hoạt, tránh tình trạng robot bị giật góc khi bấm Enable.

---

### Cách 2: Điều khiển tay kẹp Gripper bằng Script

Nếu bạn chỉ cần kiểm tra hoặc điều khiển thanh kẹp ngang J8 (động cơ DaMiao DM4310) độc lập:

```bash
# Test tự động 3 chu kỳ đóng - mở (cổng can1 cho tay trái):
python3 scripts/control_gripper.py -i can1 --action test

# Mở kẹp hoàn toàn (43 mm):
python3 scripts/control_gripper.py -i can1 --action open

# Đóng kẹp (0 mm):
python3 scripts/control_gripper.py -i can1 --action close

# Chế độ tương tác bàn phím (nhập 'o' để mở, 'c' để đóng, hoặc số mm tùy ý):
python3 scripts/control_gripper.py -i can1 --action interactive
```

---

### Cách 3: Sử dụng CLI chẩn đoán (`openarm-can-cli`)

Công cụ dòng lệnh giúp kiểm tra và gỡ lỗi đường truyền CAN bus:

```bash
# 1. Quét tìm tất cả các động cơ đang có trên đường truyền CAN:
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover

# 2. Giám sát các động cơ theo thời gian thực (hiển thị góc, tốc độ, dòng điện, nhiệt độ):
openarm-can-cli -i can0 monitor

# 3. Giám sát các động cơ cụ thể:
openarm-can-cli -i can1 monitor --id 1,2,8
```

---

### Cách 4: Lập trình điều khiển bằng C++ & Python

#### Ví dụ lập trình Python

```python
import time
import openarm_can as oa

# Kết nối cổng can0 (CAN-FD kích hoạt)
arm = oa.OpenArm("can0", True)

# Khởi tạo động cơ khớp và tay kẹp
arm.init_arm_motors([oa.MotorType.DM4310], [0x01], [0x11])
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18, oa.ControlMode.POS_FORCE)

# Kích hoạt động cơ
arm.enable_all()
time.sleep(0.1)

# Lấy đối tượng tay kẹp và điều khiển
gripper = arm.get_gripper()
# Mở kẹp: vị trí target_pos theo radian (0.0 rad: Đóng, 1.15 rad: Mở), giới hạn lực 0.15 pu
gripper.set_position(0.575, speed_rad_s=10.0, torque_pu=0.15)

time.sleep(1.0)

# Tắt an toàn
arm.disable_all()
```

#### Ví dụ lập trình C++

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

int main() {
    // Kết nối cổng can0 hỗ trợ CAN-FD
    openarm::can::socket::OpenArm arm("can0", true);

    std::vector<openarm::damiao_motor::MotorType> types = {
        openarm::damiao_motor::MotorType::DM4310
    };
    std::vector<uint32_t> send_ids = {0x01};
    std::vector<uint32_t> recv_ids = {0x11};

    arm.init_arm_motors(types, send_ids, recv_ids);
    arm.enable_all();

    // Điều khiển vị trí qua MIT Mode hoặc POS_FORCE
    // ...

    arm.disable_all();
    return 0;
}
```

---

## 6. Bảng ánh xạ Khớp & CAN ID

Hệ thống OpenArm Dual-Arm sử dụng tổng cộng 16 nút CAN phân bố trên 2 kênh CAN-FD riêng biệt:

| Khớp | Tên Khớp | CAN Interface | Send CAN ID | Feedback CAN ID | Chế độ Điều khiển | Loại Động cơ |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Left J1** | Khớp vai (Pitch) | `can1` | `0x01` | `0x11` | MIT Mode | DM4310 / DM8009 |
| **Left J2** | Khớp vai (Roll) | `can1` | `0x02` | `0x12` | MIT Mode | DM4310 |
| **Left J3** | Khớp bắp tay (Twist) | `can1` | `0x03` | `0x13` | MIT Mode | DM4310 |
| **Left J4** | Khớp khuỷu tay (Pitch) | `can1` | `0x04` | `0x14` | MIT Mode | DM4310 |
| **Left J5** | Khớp cẳng tay (Twist) | `can1` | `0x05` | `0x15` | MIT Mode | DM4310 |
| **Left J6** | Khớp cổ tay (Pitch) | `can1` | `0x06` | `0x16` | MIT Mode | DM4310 |
| **Left J7** | Khớp cổ tay (Roll) | `can1` | `0x07` | `0x17` | MIT Mode | DM4310 |
| **Left J8** | **Tay kẹp ngang (Gripper)** | `can1` | `0x08` | `0x18` | **POS_FORCE** (`0x308`) | DM4310 |
| **Right J1**| Khớp vai (Pitch) | `can0` | `0x09` | `0x19` | MIT Mode | DM4310 / DM8009 |
| **Right J2**| Khớp vai (Roll) | `can0` | `0x0A` | `0x1A` | MIT Mode | DM4310 |
| **Right J3**| Khớp bắp tay (Twist) | `can0` | `0x0B` | `0x1B` | MIT Mode | DM4310 |
| **Right J4**| Khớp khuỷu tay (Pitch) | `can0` | `0x0C` | `0x1C` | MIT Mode | DM4310 |
| **Right J5**| Khớp cẳng tay (Twist) | `can0` | `0x0D` | `0x1D` | MIT Mode | DM4310 |
| **Right J6**| Khớp cổ tay (Pitch) | `can0` | `0x0E` | `0x1E` | MIT Mode | DM4310 |
| **Right J7**| Khớp cổ tay (Roll) | `can0` | `0x0F` | `0x1F` | MIT Mode | DM4310 |
| **Right J8**| **Tay kẹp ngang (Gripper)** | `can0` | `0x10` | `0x20` | **POS_FORCE** (`0x310`) | DM4310 |

---

## 7. Xử lý sự cố thường gặp (Troubleshooting)

### 1. Lỗi `No such device` hoặc cổng CAN bị mất trên WSL2
- **Nguyên nhân**: Khi máy tính Windows vào chế độ Sleep hoặc cáp USB bị lỏng, kết nối USB trong WSL2 sẽ bị ngắt tự động.
- **Khắc phục**:
  - Cách 1: Trên giao diện Web Dashboard, bấm nút **🔌 Connect USB Robot**. Hệ thống sẽ tự động tìm và gắn lại cổng.
  - Cách 2: Mở PowerShell trên Windows và chạy lại: `usbipd attach --wsl --busid <BUSID>`. Sau đó trong WSL chạy lại lệnh bật `sudo ip link set can0 up && sudo ip link set can1 up`.

### 2. Lỗi `OSError: [Errno 98] Address already in use`
- **Nguyên nhân**: Cổng mạng `8888` hoặc `8889` đang bị chiếm dụng bởi một phiên bản server chạy ngầm trước đó.
- **Khắc phục**: Giải phóng cổng đang bị chiếm:
  ```bash
  sudo kill -9 $(lsof -t -i:8888 -i:8889)
  # Sau đó khởi động lại server:
  python3 sim/server.py
  ```

### 3. Tay kẹp Gripper (Joint 8) mở ra hết và không đóng lại được
- **Nguyên nhân**: Khi kẹp đi đến cữ chặn cơ khí cuối, động cơ gặp vật cản và kích hoạt cờ quá tải (`error_code >= 8`). Nếu hệ thống gửi mã xóa lỗi `0xFB`, firmware DaMiao sẽ chuyển động cơ về chế độ Disabled (tắt lực).
- **Khắc phục**: 
  - Code trong `sim/server.py` đã tự động xử lý gửi `0xFB` kèm `0xFC` để kích hoạt lại lực tức thời.
  - Dải góc cơ khí của kẹp là dải góc dương `0.0 rad` đến `+1.15 rad`. Nếu cơ khí bị đảo chiều, bạn chỉ cần bấm nút **⇄ Đảo chiều** trên giao diện web để hoán đổi chiều Đóng/Mở chuẩn xác.

### 4. Robot bị giật khi bấm `Enable All`
- **Nguyên nhân**: Giá trị đặt ban đầu khác với góc thực tế cơ khí của robot trước khi cấp điện.
- **Khắc phục**: Luôn bấm nút **Sync Robot State** trên thanh điều khiển trước khi bấm **Enable All** để phần mềm đồng bộ đúng góc hiện tại của robot vào thanh trượt, đảm bảo robot giữ nguyên tư thế và không bị giật.

---

## Giấy phép (License)

Dự án được phân phối dưới giấy phép [Apache License 2.0](LICENSE.txt).

Bản quyền © 2025–2026 Enactic, Inc. và nhóm phát triển OpenArm.
