# nyush-rm-vision 下位机通信协议详解

## 目录
- [1. 项目简介](#1-项目简介)
- [2. 通信架构总览](#2-通信架构总览)
- [3. Gimbal串口通信（云台）](#3-gimbal串口通信云台)
- [4. CBoard CAN通信（下位机开发板）](#4-cboard-can通信下位机开发板)
- [5. DM_IMU串口通信（达妙IMU）](#5-dm_imu串口通信达妙imu)
- [6. 数据编码与解码详解](#6-数据编码与解码详解)
- [7. 时间同步与四元数插值](#7-时间同步与四元数插值)
- [8. 配置与部署](#8-配置与部署)
- [9. 关键代码文件索引](#9-关键代码文件索引)

---

## 1. 项目简介

**nyush-rm-vision** 是同济大学SuperPower战队25赛季的RoboMaster自瞄算法开源项目。该项目采用**分布式架构**：
- **视觉侧（小电脑）**：运行识别、估计、决策算法（NUC12WSKI7，i7-1260P）
- **下位机侧（嵌入式MCU）**：运行云台控制器和执行机构（RoboMaster C板，STM32F407）

两者通过**串口**和**CAN总线**进行实时数据交换，实现视觉控制云台和自动射击的功能。

---

## 2. 通信架构总览

### 2.1 通信方式对比

| 通信接口 | 传输介质 | 波特率/带宽 | 主要功能 | 数据包大小 |
|---------|---------|-----------|---------|-----------|
| **Gimbal串口** | USB虚拟串口 | 115200 bps | 云台IMU + 控制命令 | ≤64字节 |
| **CBoard CAN** | USB2CAN/SocketCAN | 1000000 bps | 下位机IMU + 状态 + 命令 | ≤8字节 |
| **DM_IMU串口** | USB虚拟串口 | 921600 bps | 达妙IMU数据 | 57字节 |

### 2.2 数据流向

```
┌─────────────────────────────────────────────────────────────┐
│                        视觉端（小电脑）                        │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐  │
│  │  相机线程  │ → │ 识别器    │ → │ 估计器    │ → │ 决策器    │  │
│  └──────────┘   └──────────┘   └──────────┘   └──────────┘  │
│         ↓                                            ↓        │
│    时间戳 + 图像                               控制命令        │
└─────────────────────────────────────────────────────────────┘
         ↓                                            ↓
    【接收IMU数据】                              【发送控制命令】
         ↓                                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      下位机端（STM32）                         │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐  │
│  │   IMU    │ → │ 控制器    │ → │ 云台电机  │   │ 发射机构  │  │
│  └──────────┘   └──────────┘   └──────────┘   └──────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**关键流程**：
1. 下位机IMU以高频率（~1000Hz）发送四元数给视觉端
2. 视觉端接收图像，配合四元数进行坐标变换
3. 识别器识别装甲板，估计器预测目标状态
4. 决策器计算最佳瞄准位置和开火时机
5. 控制命令（yaw, pitch, 开火）发送回下位机
6. 下位机执行云台控制和射击

---

## 3. Gimbal串口通信（云台）

### 3.1 通信概述

**文件位置**：`io/gimbal/gimbal.hpp`, `io/gimbal/gimbal.cpp`

**通信特点**：
- 协议：自定义二进制协议
- 接口：USB虚拟串口（默认 `/dev/gimbal`）
- 波特率：115200 bps（默认，可配置）
- 校验：CRC16
- 线程模型：独立接收线程 + 主线程发送
- 自动重连：支持

### 3.2 数据包结构

#### 3.2.1 下位机 → 视觉（GimbalToVision）

```cpp
struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2] = {'S', 'P'};    // 包头: 'S''P' (0x53, 0x50)
    uint8_t mode;                     // 当前模式
                                      //   0: IDLE (空闲)
                                      //   1: AUTO_AIM (自瞄)
                                      //   2: SMALL_BUFF (小符)
                                      //   3: BIG_BUFF (大符)
    float q[4];                       // 四元数 [w, x, y, z] (wxyz顺序)
    float yaw;                        // Yaw角度 (弧度)
    float yaw_vel;                    // Yaw角速度 (rad/s)
    float pitch;                      // Pitch角度 (弧度)
    float pitch_vel;                  // Pitch角速度 (rad/s)
    float bullet_speed;               // 实时弹速 (m/s)
    uint16_t bullet_count;            // 子弹累计发送次数
    uint16_t crc16;                   // CRC16校验码
};
// 总大小: ≤64字节
```

**字段说明**：
- `head[2]`：固定包头，用于帧同步
- `mode`：下位机当前工作模式，由操作手拨档切换
- `q[4]`：IMU四元数，**wxyz顺序**（注意不是常见的xyzw）
- `yaw/yaw_vel`：云台yaw轴当前角度和角速度
- `pitch/pitch_vel`：云台pitch轴当前角度和角速度
- `bullet_speed`：从裁判系统获取的实时弹速
- `bullet_count`：累计发弹数（用于检测开火）
- `crc16`：整个数据包的CRC16校验

#### 3.2.2 视觉 → 下位机（VisionToGimbal）

```cpp
struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2] = {'S', 'P'};    // 包头: 'S''P'
    uint8_t mode;                     // 控制模式
                                      //   0: 不控制
                                      //   1: 控制云台但不开火
                                      //   2: 控制云台且开火
    float yaw;                        // 目标Yaw角度 (弧度)
    float yaw_vel;                    // 目标Yaw角速度 (rad/s)
    float yaw_acc;                    // 目标Yaw加速度 (rad/s²)
    float pitch;                      // 目标Pitch角度 (弧度)
    float pitch_vel;                  // 目标Pitch角速度 (rad/s)
    float pitch_acc;                  // 目标Pitch加速度 (rad/s²)
    uint16_t crc16;                   // CRC16校验码
};
// 总大小: ≤64字节
```

**字段说明**：
- `mode`：视觉控制模式
  - `0`：不控制（下位机保持原状态）
  - `1`：控制云台移动，但不开火
  - `2`：控制云台移动，且触发开火
- `yaw/yaw_vel/yaw_acc`：yaw轴的位置、速度、加速度指令（用于前馈控制）
- `pitch/pitch_vel/pitch_acc`：pitch轴的位置、速度、加速度指令

### 3.3 接收流程（gimbal.cpp:130-195）

```cpp
void Gimbal::read_thread()
{
    while (!quit_) {
        // 1. 读取包头（2字节）
        if (!read(&rx_data_, sizeof(rx_data_.head))) {
            error_count++;
            continue;
        }

        // 2. 验证包头
        if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P')
            continue;

        // 3. 记录时间戳
        auto t = std::chrono::steady_clock::now();

        // 4. 读取剩余数据
        if (!read(&rx_data_ + sizeof(rx_data_.head),
                  sizeof(rx_data_) - sizeof(rx_data_.head))) {
            error_count++;
            continue;
        }

        // 5. CRC16校验
        if (!tools::check_crc16(&rx_data_, sizeof(rx_data_))) {
            tools::logger()->debug("[Gimbal] CRC16 check failed.");
            continue;
        }

        // 6. 解析四元数（wxyz顺序）
        Eigen::Quaterniond q(
            rx_data_.q[0],  // w
            rx_data_.q[1],  // x
            rx_data_.q[2],  // y
            rx_data_.q[3]   // z
        );

        // 7. 推入四元数队列（用于时间对齐）
        queue_.push({q, t});

        // 8. 更新云台状态
        state_.yaw = rx_data_.yaw;
        state_.yaw_vel = rx_data_.yaw_vel;
        state_.pitch = rx_data_.pitch;
        state_.pitch_vel = rx_data_.pitch_vel;
        state_.bullet_speed = rx_data_.bullet_speed;
        state_.bullet_count = rx_data_.bullet_count;

        // 9. 解析模式
        switch (rx_data_.mode) {
            case 0: mode_ = GimbalMode::IDLE; break;
            case 1: mode_ = GimbalMode::AUTO_AIM; break;
            case 2: mode_ = GimbalMode::SMALL_BUFF; break;
            case 3: mode_ = GimbalMode::BIG_BUFF; break;
            default: mode_ = GimbalMode::IDLE; break;
        }

        error_count = 0;  // 重置错误计数
    }
}
```

**重点**：
- **帧同步**：通过包头'S''P'进行字节对齐
- **时间戳**：在接收到包头后立即记录，用于四元数时间对齐
- **错误处理**：连续5000次错误后触发自动重连
- **线程安全**：使用互斥锁保护状态变量

### 3.4 发送流程（gimbal.cpp:99-118）

```cpp
void Gimbal::send(
    bool control, bool fire,
    float yaw, float yaw_vel, float yaw_acc,
    float pitch, float pitch_vel, float pitch_acc)
{
    // 1. 设置控制模式
    tx_data_.mode = control ? (fire ? 2 : 1) : 0;

    // 2. 填充数据
    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.pitch_acc = pitch_acc;

    // 3. 计算CRC16（不包含crc16字段本身）
    tx_data_.crc16 = tools::get_crc16(
        &tx_data_,
        sizeof(tx_data_) - sizeof(tx_data_.crc16)
    );

    // 4. 串口发送
    try {
        serial_.write(&tx_data_, sizeof(tx_data_));
    } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to write: {}", e.what());
    }
}
```

### 3.5 断线重连机制（gimbal.cpp:200-221）

```cpp
void Gimbal::reconnect()
{
    int max_retry_count = 10;
    for (int i = 0; i < max_retry_count && !quit_; ++i) {
        tools::logger()->warn(
            "[Gimbal] Reconnecting serial, attempt {}/{}",
            i + 1, max_retry_count
        );

        try {
            serial_.close();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (...) {}

        try {
            serial_.open();  // 重新打开
            queue_.clear();  // 清空队列
            tools::logger()->info("[Gimbal] Reconnected successfully.");
            break;
        } catch (const std::exception & e) {
            tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}
```

---

## 4. CBoard CAN通信（下位机开发板）

### 4.1 通信概述

**文件位置**：`io/cboard.hpp`, `io/cboard.cpp`, `io/socketcan.hpp`

**通信特点**：
- 协议：标准CAN 2.0A
- 接口：SocketCAN（Linux内核驱动）
- 波特率：1000000 bps (1 Mbps)
- 线程模型：epoll事件驱动接收 + 守护线程
- 自动重连：支持
- 数据格式：CAN帧（最大8字节）

### 4.2 CAN ID分配

| CAN ID | 功能 | 方向 | 数据长度 |
|--------|------|------|---------|
| `0x100` (quaternion_canid) | 四元数 | 下位机→视觉 | 8字节 |
| `0x101` (bullet_speed_canid) | 弹速+模式 | 下位机→视觉 | 8字节 |
| `0xFF` (send_canid) | 控制命令 | 视觉→下位机 | 8字节 |

**注**：CAN ID可在配置文件中自定义。

### 4.3 数据包结构

#### 4.3.1 下位机 → 视觉：四元数（CAN ID 0x100）

```
CAN Frame:
├─ can_id: quaternion_canid (默认 0x100)
├─ can_dlc: 8
└─ data[8]:
    ├─ [0-1]: int16_t x * 10000  (X分量，高字节在前)
    ├─ [2-3]: int16_t y * 10000  (Y分量)
    ├─ [4-5]: int16_t z * 10000  (Z分量)
    └─ [6-7]: int16_t w * 10000  (W分量)
```

**编码方式**：
```
float → int16_t 编码:  int16_value = (int16_t)(float_value * 10000)
int16_t → 字节编码:    data[0] = high_byte, data[1] = low_byte
```

**示例**：
```cpp
// 下位机发送四元数 q = [w=1.0, x=0.0, y=0.0, z=0.0]
can_frame frame;
frame.can_id = 0x100;
frame.can_dlc = 8;
frame.data[0] = 0x00;  // x高字节 = 0
frame.data[1] = 0x00;  // x低字节 = 0
frame.data[2] = 0x00;  // y高字节 = 0
frame.data[3] = 0x00;  // y低字节 = 0
frame.data[4] = 0x00;  // z高字节 = 0
frame.data[5] = 0x00;  // z低字节 = 0
frame.data[6] = 0x27;  // w高字节 = 39 (10000/256)
frame.data[7] = 0x10;  // w低字节 = 16 (10000%256)
```

#### 4.3.2 下位机 → 视觉：弹速和模式（CAN ID 0x101）

```
CAN Frame:
├─ can_id: bullet_speed_canid (默认 0x101)
├─ can_dlc: 8
└─ data[8]:
    ├─ [0-1]: int16_t bullet_speed * 100  (弹速 m/s，高字节在前)
    ├─ [2]:   uint8_t mode                (机器人模式)
    │          0 = idle
    │          1 = auto_aim
    │          2 = small_buff
    │          3 = big_buff
    │          4 = outpost
    ├─ [3]:   uint8_t shoot_mode          (哨兵射击模式)
    │          0 = left_shoot
    │          1 = right_shoot
    │          2 = both_shoot
    └─ [4-5]: int16_t ft_angle * 10000   (无人机专用角度)
```

**编码示例**：
```cpp
// 弹速 15.0 m/s, 自瞄模式
frame.data[0] = (1500 >> 8) & 0xFF;  // 高字节 = 5
frame.data[1] = 1500 & 0xFF;         // 低字节 = 220
frame.data[2] = 1;                   // auto_aim
frame.data[3] = 0;                   // left_shoot
frame.data[4] = 0;                   // ft_angle高字节
frame.data[5] = 0;                   // ft_angle低字节
```

#### 4.3.3 视觉 → 下位机：控制命令（CAN ID 0xFF）

```
CAN Frame:
├─ can_id: send_canid (默认 0xFF)
├─ can_dlc: 8
└─ data[8]:
    ├─ [0]:   uint8_t control              (是否控制云台: 0/1)
    ├─ [1]:   uint8_t shoot                (是否开火: 0/1)
    ├─ [2-3]: int16_t yaw * 10000         (目标Yaw角度，弧度)
    ├─ [4-5]: int16_t pitch * 10000       (目标Pitch角度，弧度)
    └─ [6-7]: int16_t horizon_dist * 10000 (水平距离，无人机专用)
```

**编码方式**（cboard.cpp:47-66）：
```cpp
void CBoard::send(Command command) const
{
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;

    // 控制标志位
    frame.data[0] = (command.control) ? 1 : 0;
    frame.data[1] = (command.shoot) ? 1 : 0;

    // yaw角度（int16_t * 10000，大端序）
    frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;   // 高字节
    frame.data[3] = (int16_t)(command.yaw * 1e4) & 0xFF; // 低字节

    // pitch角度
    frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
    frame.data[5] = (int16_t)(command.pitch * 1e4) & 0xFF;

    // 水平距离（无人机专用）
    frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
    frame.data[7] = (int16_t)(command.horizon_distance * 1e4) & 0xFF;

    can_.write(&frame);
}
```

### 4.4 接收解析（cboard.cpp:68-103）

```cpp
void CBoard::callback(const can_frame & frame)
{
    auto timestamp = std::chrono::steady_clock::now();

    // 解析四元数帧
    if (frame.can_id == quaternion_canid_) {
        // 大端序解析：高字节在前
        auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
        auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
        auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
        auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

        // 四元数有效性检查（模长应为1）
        if (std::abs(x*x + y*y + z*z + w*w - 1) > 1e-2) {
            tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
            return;
        }

        // 推入队列（wxyz顺序）
        queue_.push({{w, x, y, z}, timestamp});
    }

    // 解析弹速和模式帧
    else if (frame.can_id == bullet_speed_canid_) {
        bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
        mode = Mode(frame.data[2]);
        shoot_mode = ShootMode(frame.data[3]);
        ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

        // 限制日志输出频率为1Hz
        static auto last_log_time = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();

        if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
            tools::logger()->info(
                "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}",
                bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode]
            );
            last_log_time = now;
        }
    }
}
```

### 4.5 SocketCAN驱动（socketcan.hpp）

**核心特性**：
- **epoll事件驱动**：高效监听CAN消息
- **双线程架构**：
  - 接收线程：epoll_wait循环接收CAN帧
  - 守护线程：定期检查连接状态，断线自动重连
- **非阻塞IO**：使用 `MSG_DONTWAIT` 标志

**接收循环**（socketcan.hpp:135-146）：
```cpp
void SocketCAN::read()
{
    // epoll等待事件（2ms超时）
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1)
        throw std::runtime_error("Error waiting for events!");

    for (int i = 0; i < num_events; i++) {
        // 非阻塞接收CAN帧
        ssize_t num_bytes = recv(socket_fd_, &frame_,
                                  sizeof(can_frame), MSG_DONTWAIT);
        if (num_bytes == -1)
            throw std::runtime_error("Error reading from SocketCAN!");

        // 调用回调函数处理数据
        rx_handler_(frame_);
    }
}
```

**守护线程**（socketcan.hpp:38-49）：
```cpp
daemon_thread_ = std::thread{[this] {
    while (!quit_) {
        std::this_thread::sleep_for(100ms);

        // 连接正常，继续等待
        if (ok_) continue;

        // 连接异常，重启接收线程
        if (read_thread_.joinable())
            read_thread_.join();

        close();
        try_open();  // 尝试重新打开
    }
}};
```

---

## 5. DM_IMU串口通信（达妙IMU）

### 5.1 通信概述

**文件位置**：`io/dm_imu/dm_imu.hpp`, `io/dm_imu/dm_imu.cpp`

**通信特点**：
- 协议：达妙IMU私有协议
- 接口：USB虚拟串口（默认 `/dev/ttyACM0`）
- 波特率：921600 bps
- 数据位：8
- 停止位：1
- 奇偶校验：无
- 校验：CRC16（每帧独立校验）
- 数据包：57字节（3个子帧）

### 5.2 数据包结构

```cpp
struct __attribute__((packed)) IMU_Receive_Frame
{
    // === 子帧1：加速度数据 ===
    uint8_t FrameHeader1;       // 0x55
    uint8_t flag1;              // 0xAA
    uint8_t slave_id1;          // 0x01
    uint8_t reg_acc;            // 寄存器地址
    uint32_t accx_u32;          // X轴加速度（以uint32存储float）
    uint32_t accy_u32;          // Y轴加速度
    uint32_t accz_u32;          // Z轴加速度
    uint16_t crc1;              // 子帧1的CRC16
    uint8_t FrameEnd1;          // 帧尾

    // === 子帧2：陀螺仪数据 ===
    uint8_t FrameHeader2;       // 0x55
    uint8_t flag2;              // 0xAA
    uint8_t slave_id2;          // 0x01
    uint8_t reg_gyro;           // 寄存器地址
    uint32_t gyrox_u32;         // X轴角速度
    uint32_t gyroy_u32;         // Y轴角速度
    uint32_t gyroz_u32;         // Z轴角速度
    uint16_t crc2;              // 子帧2的CRC16
    uint8_t FrameEnd2;          // 帧尾

    // === 子帧3：欧拉角数据 ===
    uint8_t FrameHeader3;       // 0x55
    uint8_t flag3;              // 0xAA
    uint8_t slave_id3;          // 0x01
    uint8_t reg_euler;          // 寄存器地址（R-P-Y顺序）
    uint32_t roll_u32;          // 滚转角（以uint32存储float）
    uint32_t pitch_u32;         // 俯仰角
    uint32_t yaw_u32;           // 偏航角
    uint16_t crc3;              // 子帧3的CRC16
    uint8_t FrameEnd3;          // 帧尾
};
// 总大小: 3 * 19字节 = 57字节
```

### 5.3 数据解析

**float转换**：
达妙IMU使用 `uint32_t` 直接存储 `float` 的二进制表示（IEEE 754），需要通过指针转换：

```cpp
// uint32_t → float 转换
float acc_x = *(float*)(&receive_data.accx_u32);
float acc_y = *(float*)(&receive_data.accy_u32);
float acc_z = *(float*)(&receive_data.accz_u32);

float gyro_x = *(float*)(&receive_data.gyrox_u32);
float gyro_y = *(float*)(&receive_data.gyroy_u32);
float gyro_z = *(float*)(&receive_data.gyroz_u32);

float roll = *(float*)(&receive_data.roll_u32);
float pitch = *(float*)(&receive_data.pitch_u32);
float yaw = *(float*)(&receive_data.yaw_u32);
```

### 5.4 数据输出结构

```cpp
typedef struct {
    float accx;      // X轴加速度 (m/s²)
    float accy;      // Y轴加速度
    float accz;      // Z轴加速度
    float gyrox;     // X轴角速度 (rad/s)
    float gyroy;     // Y轴角速度
    float gyroz;     // Z轴角速度
    float roll;      // 滚转角 (rad)
    float pitch;     // 俯仰角 (rad)
    float yaw;       // 偏航角 (rad)
} IMU_Data;
```

---

## 6. 数据编码与解码详解

### 6.1 整数定点化编码

为了在有限的字节内传输浮点数，采用**定点数**编码方式：

| 数据类型 | 缩放因子 | int16范围 | float范围 | 精度 |
|---------|---------|-----------|-----------|------|
| 四元数分量 | 10000 | -32768~32767 | -3.28~3.28 | 0.0001 |
| 角度（弧度） | 10000 | -32768~32767 | -3.28~3.28 | 0.0001 |
| 弹速（m/s） | 100 | -32768~32767 | -327~327 | 0.01 |

**编码公式**：
```
int16_value = (int16_t)(float_value * scale_factor)
```

**解码公式**：
```
float_value = (float)int16_value / scale_factor
```

### 6.2 字节序（大端序）

CAN通信和Gimbal串口均使用**大端序**（Big-Endian）编码int16：

```cpp
// 编码（float → 字节）
int16_t value = (int16_t)(angle * 10000);
data[0] = (value >> 8) & 0xFF;  // 高字节
data[1] = value & 0xFF;         // 低字节

// 解码（字节 → float）
int16_t value = (int16_t)(data[0] << 8 | data[1]);
float angle = value / 10000.0;
```

**示例**：
```
angle = 0.5236 rad (30°)
value = 5236
data[0] = 0x14 (20)
data[1] = 0x74 (116)

接收端:
value = 0x14 << 8 | 0x74 = 5236
angle = 5236 / 10000.0 = 0.5236 rad
```

### 6.3 CRC16校验

**算法**：CRC-16/MODBUS（多项式 0x8005）

**校验范围**：
- Gimbal串口：整个数据包（不包含crc16字段）
- DM_IMU：每个子帧独立校验

**代码示例**（tools/crc.hpp）：
```cpp
// 计算CRC16
uint16_t crc16 = tools::get_crc16(data, length);

// 验证CRC16
bool valid = tools::check_crc16(data, total_length);
```

---

## 7. 时间同步与四元数插值

### 7.1 问题背景

视觉处理存在延迟：
- **图像采集延迟**：曝光时间（~2-3ms）
- **传输延迟**：USB传输（~1-2ms）
- **处理延迟**：识别+估计（~5-10ms）
- **通信延迟**：串口/CAN发送（~1ms）

因此，当视觉算法处理某帧图像时，需要获取**该图像曝光时刻**的IMU姿态，而非当前时刻的姿态。

### 7.2 解决方案：时间对齐 + SLERP插值

#### 7.2.1 四元数队列缓存

```cpp
// Gimbal类
tools::ThreadSafeQueue<
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>
> queue_{1000};  // 缓存1000个四元数+时间戳对

// CBoard类
tools::ThreadSafeQueue<IMUData> queue_;  // IMU数据队列

struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
};
```

#### 7.2.2 四元数插值算法（SLERP）

**Gimbal实现**（gimbal.cpp:64-78）：
```cpp
Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
    while (true) {
        // 1. 弹出队头四元数
        auto [q_a, t_a] = queue_.pop();

        // 2. 查看队头的下一个四元数（不弹出）
        auto [q_b, t_b] = queue_.front();

        // 3. 计算时间差
        auto t_ab = tools::delta_time(t_a, t_b);  // t_b - t_a
        auto t_ac = tools::delta_time(t_a, t);    // t - t_a

        // 4. 计算插值系数
        auto k = t_ac / t_ab;

        // 5. SLERP球面线性插值
        Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

        // 6. 如果t在[t_a, t_b]区间内，返回插值结果
        if (t_a < t && t <= t_b) {
            return q_c;
        }

        // 否则继续弹出队列，寻找正确的时间区间
    }
}
```

**CBoard实现**（cboard.cpp:22-45）：
```cpp
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
    // 1. 如果data_behind_时间早于timestamp，更新data_ahead_
    if (data_behind_.timestamp < timestamp) {
        data_ahead_ = data_behind_;
    }

    // 2. 弹出队列，找到timestamp之后的第一个数据点
    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp) break;
        data_ahead_ = data_behind_;
    }

    // 3. 归一化四元数
    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();

    // 4. 计算插值系数
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;
    auto k = t_ac / t_ab;

    // 5. SLERP插值
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
}
```

#### 7.2.3 SLERP插值原理

SLERP（Spherical Linear Interpolation）是四元数的球面线性插值：

```
q(t) = q_a * sin((1-k)θ) / sin(θ) + q_b * sin(kθ) / sin(θ)

其中：
- θ = arccos(q_a · q_b)
- k ∈ [0, 1] 为插值系数
```

Eigen库的实现：
```cpp
Eigen::Quaterniond q_c = q_a.slerp(k, q_b);
```

**优点**：
- 保持四元数的单位模长
- 插值路径为测地线（最短路径）
- 角速度恒定

### 7.3 预测时间补偿

根据readme.md，还需要考虑**未来预测时间**：

```
预测时间 = 图像延迟 + 处理延迟 + 通信延迟 + 控制延迟
         ≈ 15ms (通过调试参数调整)
```

在实际使用中：
```cpp
// 获取图像曝光时刻的四元数
auto t_exposure = image_timestamp;
auto q_exposure = gimbal.q(t_exposure);

// 预测未来时刻的姿态（用于弹道补偿）
auto t_predict = t_exposure + std::chrono::milliseconds(15);
// ... 根据角速度推算未来姿态
```

---

## 8. 配置与部署

### 8.1 配置文件示例

**标准配置**（configs/standard3.yaml）：

```yaml
#####-----cboard参数-----#####
quaternion_canid: 0x100          # 四元数接收CAN ID
bullet_speed_canid: 0x101        # 弹速/模式接收CAN ID
send_canid: 0xff                 # 命令发送CAN ID
can_interface: "can0"            # CAN接口名称

#####-----gimbal参数-----#####
com_port: "/dev/gimbal"          # 云台串口设备（通过udev映射）
yaw_kp: 0                        # Yaw轴PD参数（未使用）
yaw_kd: 0
pitch_kp: 0                      # Pitch轴PD参数（未使用）
pitch_kd: 0
```

### 8.2 串口设备映射（udev规则）

**问题**：USB串口设备名（`/dev/ttyACM0`）会随插拔顺序变化。

**解决**：使用udev规则创建固定符号链接。

#### 步骤1：授予用户权限
```bash
sudo usermod -a -G dialout $USER
```

#### 步骤2：获取设备信息
```bash
udevadm info -a -n /dev/ttyACM0 | grep -E '(serial|idVendor|idProduct)'
```

示例输出：
```
ATTRS{idVendor}=="0483"
ATTRS{idProduct}=="5740"
ATTRS{serial}=="206A356C3137"
```

#### 步骤3：创建udev规则
```bash
sudo touch /etc/udev/rules.d/99-usb-serial.rules
```

编辑文件，写入：
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="206A356C3137", SYMLINK+="gimbal"
```

**说明**：
- `SUBSYSTEM=="tty"`：仅匹配串口设备
- `ATTRS{...}`：根据VID、PID、序列号精确匹配
- `SYMLINK+="gimbal"`：创建符号链接 `/dev/gimbal` → `/dev/ttyACM0`

#### 步骤4：重新加载规则
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 步骤5：验证
```bash
ls -l /dev/gimbal
# 输出: lrwxrwxrwx 1 root root 7 ... /dev/gimbal -> ttyACM0
```

### 8.3 CAN接口配置

#### 步骤1：启用SocketCAN模块
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan
```

#### 步骤2：配置CAN接口
```bash
# 设置波特率为1Mbps
sudo ip link set can0 type can bitrate 1000000

# 启动接口
sudo ip link set can0 up

# 查看状态
ip -details link show can0
```

#### 步骤3：自动启动（udev规则）
创建 `/etc/udev/rules.d/99-can-up.rules`：
```
ACTION=="add", KERNEL=="can0", RUN+="/sbin/ip link set can0 up type can bitrate 1000000"
ACTION=="add", KERNEL=="can1", RUN+="/sbin/ip link set can1 up type can bitrate 1000000"
```

#### 步骤4：测试CAN通信
```bash
# 发送测试帧
cansend can0 100#0123456789ABCDEF

# 监听所有CAN消息
candump can0
```

### 8.4 程序自启动

#### 方法1：systemd服务（推荐）
创建 `/etc/systemd/system/sp_vision.service`：
```ini
[Unit]
Description=SuperPower Vision Service
After=network.target

[Service]
Type=simple
User=rm
WorkingDirectory=/home/rm/Desktop/sp_vision_25
ExecStart=/home/rm/Desktop/sp_vision_25/build/auto_aim configs/standard3.yaml
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用服务：
```bash
sudo systemctl daemon-reload
sudo systemctl enable sp_vision.service
sudo systemctl start sp_vision.service

# 查看状态
sudo systemctl status sp_vision.service
```

#### 方法2：Desktop Entry（开机自启GUI）
创建 `~/.config/autostart/sp_vision.desktop`：
```ini
[Desktop Entry]
Type=Application
Exec=/home/rm/Desktop/sp_vision_25/autostart.sh
Name=sp_vision
```

`autostart.sh` 内容：
```bash
#!/bin/bash
cd /home/rm/Desktop/sp_vision_25
screen -dmS sp_vision ./build/auto_aim configs/standard3.yaml
```

赋予执行权限：
```bash
chmod +x autostart.sh
```

### 8.5 测试程序

编译后运行测试程序：

```bash
# Gimbal串口测试
./build/gimbal_test configs/standard3.yaml

# CBoard CAN测试
./build/cboard_test configs/example.yaml

# 达妙IMU测试
./build/dm_test

# 云台响应测试
./build/gimbal_response_test configs/standard3.yaml

# 开火测试
./build/fire_test configs/standard3.yaml
```

---

## 9. 关键代码文件索引

### 9.1 通信协议定义

| 文件路径 | 功能描述 | 行数 |
|---------|---------|------|
| `io/gimbal/gimbal.hpp` | Gimbal串口协议结构体定义 | 105 |
| `io/command.hpp` | 视觉→下位机命令结构体 | 16 |
| `io/cboard.hpp` | CBoard通信接口定义 | 71 |
| `io/dm_imu/dm_imu.hpp` | 达妙IMU数据帧定义 | 96 |

### 9.2 通信实现

| 文件路径 | 功能描述 | 行数 |
|---------|---------|------|
| `io/gimbal/gimbal.cpp` | Gimbal串口通信实现 | 222 |
| `io/cboard.cpp` | CBoard CAN通信实现 | 120 |
| `io/socketcan.hpp` | SocketCAN驱动（头文件实现） | 158 |
| `io/dm_imu/dm_imu.cpp` | 达妙IMU通信实现 | 131 |

### 9.3 串口库（第三方）

| 文件路径 | 功能描述 |
|---------|---------|
| `io/serial/include/serial/serial.h` | 串口库API定义 |
| `io/serial/src/serial.cc` | 串口库实现 |
| `io/serial/src/impl/unix.cc` | Linux平台串口实现 |

### 9.4 辅助工具

| 文件路径 | 功能描述 |
|---------|---------|
| `tools/crc.hpp` | CRC16校验算法 |
| `tools/logger.hpp` | 日志系统 |
| `tools/yaml.hpp` | YAML配置文件解析 |
| `tools/thread_safe_queue.hpp` | 线程安全队列 |
| `tools/math_tools.hpp` | 数学工具（时间转换等） |

### 9.5 测试程序

| 文件路径 | 功能描述 |
|---------|---------|
| `tests/gimbal_test.cpp` | Gimbal通信测试 |
| `tests/cboard_test.cpp` | CBoard通信测试 |
| `tests/dm_test.cpp` | 达妙IMU测试 |
| `tests/gimbal_response_test.cpp` | 云台响应延迟测试 |
| `tests/fire_test.cpp` | 开火功能测试 |

### 9.6 配置文件

| 文件路径 | 用途 |
|---------|------|
| `configs/standard3.yaml` | 3号步兵配置 |
| `configs/standard4.yaml` | 4号步兵配置 |
| `configs/sentry.yaml` | 哨兵配置 |
| `configs/uav.yaml` | 无人机配置 |
| `configs/example.yaml` | 示例配置 |

---

## 附录A：数据包二进制示例

### A.1 Gimbal串口数据包（视觉→下位机）

**场景**：控制云台移动到yaw=0.5rad, pitch=0.3rad，且开火

```
偏移  | 字段          | 值（十六进制） | 值（解释）
------|--------------|--------------|-------------
0x00  | head[0]      | 0x53         | 'S'
0x01  | head[1]      | 0x50         | 'P'
0x02  | mode         | 0x02         | 控制且开火
0x03  | yaw          | 0x3F 00 00 00| 0.5 (float)
0x07  | yaw_vel      | 0x40 00 00 00| 2.0 (float)
0x0B  | yaw_acc      | 0x00 00 00 00| 0.0 (float)
0x0F  | pitch        | 0x3E 99 99 9A| 0.3 (float)
0x13  | pitch_vel    | 0x3F 80 00 00| 1.0 (float)
0x17  | pitch_acc    | 0x00 00 00 00| 0.0 (float)
0x1B  | crc16        | 0xXX XX      | CRC校验
```

### A.2 CBoard CAN数据包（视觉→下位机）

**场景**：控制云台yaw=0.1rad, pitch=-0.05rad，且开火

```
CAN帧:
├─ can_id: 0xFF
├─ can_dlc: 8
└─ data:
    [0] = 0x01        // control = true
    [1] = 0x01        // shoot = true
    [2] = 0x03        // yaw高字节 = 1000/256 = 3
    [3] = 0xE8        // yaw低字节 = 1000%256 = 232
    [4] = 0xFE        // pitch高字节 = -500/256 = -2 (补码0xFE)
    [5] = 0x0C        // pitch低字节 = -500%256 = 12
    [6] = 0x00        // horizon_dist高字节
    [7] = 0x00        // horizon_dist低字节
```

---

## 附录B：常见问题排查

### B.1 串口连接失败

**现象**：程序启动时报错 `Failed to open serial`

**排查步骤**：
1. 检查设备是否存在：`ls -l /dev/gimbal`
2. 检查权限：`groups $USER` 确认是否在 `dialout` 组
3. 检查设备是否被占用：`lsof /dev/gimbal`
4. 查看内核日志：`dmesg | tail -20`

### B.2 CAN通信无数据

**现象**：`candump can0` 无输出

**排查步骤**：
1. 检查接口状态：`ip -details link show can0`
2. 检查波特率是否匹配：下位机和上位机必须一致（1Mbps）
3. 检查物理连接：CAN_H、CAN_L、GND是否正确连接
4. 检查终端电阻：120Ω终端电阻是否焊接

### B.3 四元数校验失败

**现象**：日志输出 `Invalid q: w x y z`

**原因**：
- 四元数模长不为1（`x²+y²+z²+w²≠1`）
- CAN传输错误或数据损坏

**解决**：
- 检查CAN线缆屏蔽
- 降低CAN波特率（500kbps）
- 检查下位机数据打包逻辑

### B.4 时间对齐异常

**现象**：云台跟踪抖动，延迟大

**原因**：
- 四元数队列为空
- 时间戳不同步

**解决**：
- 增大队列大小（默认1000）
- 确保下位机和视觉端使用同一时钟源
- 检查IMU数据发送频率（应>500Hz）

---

## 附录C：通信性能指标

### C.1 实测延迟

| 环节 | 延迟（ms） | 备注 |
|-----|-----------|------|
| IMU数据发送频率 | 1.0 | 1000Hz |
| 串口传输延迟 | 0.5-1.0 | 115200 bps |
| CAN传输延迟 | 0.2-0.5 | 1Mbps |
| 四元数插值计算 | <0.1 | SLERP算法 |
| 控制命令发送 | 0.5-1.0 | - |
| **总通信延迟** | **2-3ms** | - |

### C.2 带宽占用

**Gimbal串口**：
- 数据包大小：≤64字节
- 发送频率：视觉侧主动发送，约100Hz
- 带宽占用：64B × 8bit × 100Hz = 51.2 kbps
- 占用率：51.2/115.2 = **44.4%**

**CBoard CAN**：
- 四元数帧：8字节 × 1000Hz = 8 kB/s = 64 kbps
- 弹速帧：8字节 × 100Hz = 0.8 kB/s = 6.4 kbps
- 控制帧：8字节 × 100Hz = 0.8 kB/s = 6.4 kbps
- 总带宽：76.8 kbps
- 占用率：76.8/1000 = **7.68%**

---

## 附录D：未来优化方向

### D.1 通信协议优化

1. **数据压缩**：
   - 四元数仅需3个分量（第4个可计算）
   - 使用int8编码低精度数据

2. **协议升级**：
   - 增加数据包序列号（检测丢包）
   - 增加时间戳字段（同步时钟）

3. **多包传输**：
   - 分离高频数据（IMU 1000Hz）和低频数据（弹速 10Hz）
   - 减少无效数据传输

### D.2 时间同步优化

1. **PTP协议**：
   - 使用IEEE 1588 PTP精确时间同步
   - 时钟同步精度<1μs

2. **Kalman滤波**：
   - 融合多源IMU数据
   - 平滑四元数抖动

### D.3 可靠性优化

1. **心跳机制**：
   - 定期发送心跳包
   - 超时自动重启通信

2. **数据校验**：
   - 使用CRC32或MD5
   - 增加数据包序列号

---

## 版权声明

本文档基于 [nyush-rm-vision](https://github.com/TongjiSuperPower/sp_vision_25) 开源项目分析整理。

项目版权归同济大学SuperPower战队所有。

文档编写日期：2026-02-08
