# 'SP'协议整合计划: nyush-rm-vision ↔ nyush-rm-control

## 概述

本文档详细说明了如何将视觉系统(nyush-rm-vision)和控制系统(nyush-rm-control)通过'SP'协议连接起来。

**文档版本**: 1.1 (修正版)
**日期**: 2026-02-08
**状态**: 实施计划
**修订**: 修复了模式映射、数据源、CRC兼容性等关键问题

---

## ⚠️ 实施前必读

### 关键注意事项

1. **CRC16算法兼容性**: 控制端和视觉端的CRC16算法**可能不兼容**,必须在实施前验证!
2. **接收缓冲区大小**: 必须更新 `VISION_RECV_SIZE` 和 `VISION_SEND_SIZE` 宏定义
3. **INS数据访问**: `INS_t INS` 是静态变量,必须通过新增函数访问
4. **工作模式映射**: 控制端枚举值从0开始,视觉端期望值从0开始但含义不同,需要正确映射

详细问题清单请参见: `control-integration-issues.md`

---

## 目录

- [问题分析](#问题分析)
- [通信接口配置](#通信接口配置)
- [协议数据映射](#协议数据映射)
- [实施步骤](#实施步骤)
- [验证策略](#验证策略)
- [关键文件清单](#关键文件清单)
- [代码示例](#代码示例)
- [实施时间线](#实施时间线)

---

## 问题分析

### 当前状态

视觉系统(nyush-rm-vision)和控制系统(nyush-rm-control)使用了**完全不兼容的通信协议**:

| 系统 | 协议类型 | 帧头 | 数据格式 | 姿态表示 |
|------|---------|------|---------|---------|
| **视觉端** | 自定义'SP'协议 | `'S','P'` | 二进制结构体 + CRC16 | 四元数 + 速度 |
| **控制端** | SeaSky协议 | `0xA5` | 标志位编码 + CRC16 | 欧拉角 |

### 推荐方案

**修改控制系统采用视觉系统的'SP'协议**

**选择理由:**
1. 视觉协议更现代,包含四元数+速度+加速度数据
2. 控制端已有四元数数据(`INS_t.q[4]`),无需额外计算
3. 仅需修改控制端,视觉端保持不变
4. 新协议支持速度前馈控制,可提升性能
5. 'SP'协议更简单高效,无需复杂的标志位编解码

---

## 通信接口配置

### 硬件连接方式

**推荐使用: USB虚拟串口(VCP模式)**

C板(STM32控制板)和上位机(视觉系统NUC)之间的通信**必须走USB串口**。

**硬件连接:**
```
┌─────────────────┐                    ┌──────────────────┐
│  C板 (STM32)    │   USB Type-C线     │  NUC (视觉系统)  │
│  RoboMaster     │◄──────────────────►│  运行视觉算法    │
│  Type-C板       │   USB CDC (VCP)    │                  │
└─────────────────┘                    └──────────────────┘
```

**接口说明:**
- **物理接口**: C板上的USB Type-C接口
- **协议**: USB CDC (Communication Device Class)
- **Linux设备**: 通常映射为 `/dev/ttyACM0` 或 `/dev/ttyUSB0`
- **udev映射**: 视觉端可配置udev规则将其映射为 `/dev/gimbal`

### 配置方法

**控制端配置 (robot_def.h):**
```c
// 使用USB虚拟串口模式
#define VISION_USE_VCP

// 如需使用UART模式(不推荐)
// #define VISION_USE_UART
```

**视觉端配置 (configs/*.yaml):**
```yaml
# 串口配置
com_port: "/dev/gimbal"  # udev映射后的设备名,实际为USB虚拟串口
```

**udev规则示例 (/etc/udev/rules.d/99-rm-devices.rules):**
```bash
# RoboMaster C板 USB虚拟串口
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="gimbal", MODE="0666"
```

应用udev规则:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 通信参数

**USB虚拟串口参数:**
- **波特率**: 理论上任意(USB CDC不受限),建议配置为 921600 bps
- **数据位**: 8
- **停止位**: 1
- **奇偶校验**: None
- **流控**: None

**性能指标:**
- **发送频率**: 控制端 → 视觉端: 20Hz (可优化到100Hz)
- **接收频率**: 视觉端 → 控制端: 按需 (通常10-100Hz)
- **延迟**: < 5ms (USB全速模式)
- **带宽**: USB 2.0 全速 (12 Mbps), 远超协议需求

---

## 协议数据映射

### 控制端 → 视觉端 (GimbalToVision)

控制系统需要发送以下结构体:

```c
#pragma pack(1)
typedef struct {
    uint8_t head[2];        // 'S', 'P' 帧头
    uint8_t mode;           // 0:空闲, 1:自瞄, 2:小符, 3:大符
    float q[4];             // 四元数 (wxyz顺序)
    float yaw;              // 弧度
    float yaw_vel;          // 角速度 rad/s
    float pitch;            // 弧度
    float pitch_vel;        // 角速度 rad/s
    float bullet_speed;     // 弹速 m/s
    uint16_t bullet_count;  // 累计弹丸数
    uint16_t crc16;         // CRC16校验
} GimbalToVision_s;  // 总共44字节
#pragma pack()
```

**字段说明:**

| 字段 | 类型 | 大小 | 说明 | 控制系统数据来源 |
|------|------|------|------|-----------------|
| `head[2]` | uint8_t | 2B | 帧头: `{'S','P'}` | 新增常量 |
| `mode` | uint8_t | 1B | 工作模式 | 从 `Work_Mode_e` 映射 |
| `q[4]` | float | 16B | 四元数 wxyz | **通过`INS_GetQuaternion()`获取** |
| `yaw` | float | 4B | 偏航角(弧度) | `send_data.yaw` (度→弧度) |
| `yaw_vel` | float | 4B | 偏航角速度 | **通过`INS_GetGyro()`获取** `gyro[2]` |
| `pitch` | float | 4B | 俯仰角(弧度) | `send_data.pitch` (度→弧度) |
| `pitch_vel` | float | 4B | 俯仰角速度 | **通过`INS_GetGyro()`获取** `gyro[0]` |
| `bullet_speed` | float | 4B | 实时弹速 | `send_data.bullet_speed` |
| `bullet_count` | uint16_t | 2B | 累计弹丸数 | 裁判系统 (见后文说明) |
| `crc16` | uint16_t | 2B | CRC16校验 | 使用 `crc_16()` 函数 |

**工作模式映射 (CRITICAL!):**
```c
// Work_Mode_e (控制端) → mode (视觉端期望)
// 控制端枚举: VISION_MODE_AIM=0, VISION_MODE_SMALL_BUFF=1, VISION_MODE_BIG_BUFF=2
// 视觉端期望: 0=IDLE, 1=AUTO_AIM, 2=SMALL_BUFF, 3=BIG_BUFF

switch (send_data.work_mode) {
    case VISION_MODE_AIM:        send_packet.mode = 1; break; // 0→1
    case VISION_MODE_SMALL_BUFF: send_packet.mode = 2; break; // 1→2
    case VISION_MODE_BIG_BUFF:   send_packet.mode = 3; break; // 2→3
    default:                     send_packet.mode = 0; break; // 其他→0 (IDLE)
}
```

### 视觉端 → 控制端 (VisionToGimbal)

控制系统需要接收以下结构体:

```c
#pragma pack(1)
typedef struct {
    uint8_t head[2];        // 'S', 'P' 帧头
    uint8_t mode;           // 0:不控制, 1:控制云台, 2:控制云台+开火
    float yaw;              // 弧度
    float yaw_vel;          // 角速度 rad/s
    float yaw_acc;          // 角加速度 rad/s²
    float pitch;            // 弧度
    float pitch_vel;        // 角速度 rad/s
    float pitch_acc;        // 角加速度 rad/s²
    uint16_t crc16;         // CRC16校验
} VisionToGimbal_s;  // 总共29字节
#pragma pack()
```

**字段说明:**

| 字段 | 类型 | 大小 | 说明 | 映射到控制端 |
|------|------|------|------|-------------|
| `head[2]` | uint8_t | 2B | 帧头: `{'S','P'}` | 验证用 |
| `mode` | uint8_t | 1B | 控制模式 | → `fire_mode` |
| `yaw` | float | 4B | 目标偏航角 | 弧度→度,传给云台控制器 |
| `yaw_vel` | float | 4B | 目标偏航角速度 | **新增!** 前馈控制 |
| `yaw_acc` | float | 4B | 目标偏航角加速度 | **新增!** 前馈控制 |
| `pitch` | float | 4B | 目标俯仰角 | 弧度→度,传给云台控制器 |
| `pitch_vel` | float | 4B | 目标俯仰角速度 | **新增!** 前馈控制 |
| `pitch_acc` | float | 4B | 目标俯仰角加速度 | **新增!** 前馈控制 |
| `crc16` | uint16_t | 2B | CRC16校验 | 验证用 |

**模式映射:**
```c
// mode → Vision_Recv_s
0 → NO_FIRE     // 不控制
1 → AUTO_AIM    // 控制云台
2 → AUTO_FIRE   // 控制云台+开火
```

**映射到控制端现有结构:**
- `target_state`: 根据数据新鲜度/有效性推断 (新数据→READY_TO_FIRE)
- `target_type`: 设置为默认值 `NO_TARGET_NUM` (视觉端暂不提供)

---

## 实施步骤

### 阶段0: 实施前准备 (CRITICAL!)

#### 0.1 更新宏定义

**文件**: `modules/master_machine/master_process.h`

**必须修改:**
```c
// 原来的定义 (错误!)
// #define VISION_RECV_SIZE 18u
// #define VISION_SEND_SIZE 36u

// 新的定义 (正确)
#define VISION_RECV_SIZE 29u  // VisionToGimbal大小
#define VISION_SEND_SIZE 44u  // GimbalToVision大小
```

#### 0.2 验证CRC16兼容性

**⚠️ 关键步骤 - 不可跳过!**

控制端和视觉端的CRC16查找表不同,必须验证算法是否兼容:

**测试方法:**
```c
// 在控制端添加测试代码
uint8_t test_data[] = {'S', 'P', 0x01, 0x00, 0x00, 0x80, 0x3F}; // 示例数据
uint16_t crc = crc_16(test_data, sizeof(test_data));
LOGINFO("[CRC Test] Control CRC16: 0x%04X", crc);
// 预期结果: 应该与视觉端计算的CRC完全一致
```

在视觉端使用相同数据:
```cpp
uint8_t test_data[] = {'S', 'P', 0x01, 0x00, 0x00, 0x80, 0x3F};
uint16_t crc = tools::get_crc16(test_data, sizeof(test_data));
printf("Vision CRC16: 0x%04X\n", crc);
```

**如果CRC不一致:** 需要统一CRC算法!可以:
1. 将视觉端的CRC表移植到控制端 (推荐)
2. 或修改视觉端使用控制端的CRC算法

#### 0.3 添加必要的头文件包含

**文件**: `modules/master_machine/master_process.c`

在文件开头添加:
```c
#include "ins_task.h"      // INS_GetQuaternion(), INS_GetGyro()
#include <string.h>         // memcpy()
#include <math.h>           // fabsf()
#include "bsp_log.h"        // LOGINFO, LOGWARNING
```

---

### 阶段1: 协议层替换 (控制端)

**目标**: 在控制端实现'SP'协议的收发功能

**修改文件:**
- `modules/master_machine/master_process.h` (协议定义)
- `modules/master_machine/master_process.c` (收发实现)

#### 1.1 添加协议结构体定义

**文件**: `master_process.h` (第103行之后)

```c
// ==================== SP协议定义 ====================
#define VISION_SP_HEADER_1 'S'
#define VISION_SP_HEADER_2 'P'

// 更新缓冲区大小宏定义
#undef VISION_RECV_SIZE
#undef VISION_SEND_SIZE
#define VISION_RECV_SIZE 29u
#define VISION_SEND_SIZE 44u

#pragma pack(1)
// 控制端发送给视觉端的数据包
typedef struct {
    uint8_t head[2];        // 'S', 'P'
    uint8_t mode;
    float q[4];             // wxyz顺序
    float yaw, yaw_vel, pitch, pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision_s;

// 视觉端发送给控制端的数据包
typedef struct {
    uint8_t head[2];        // 'S', 'P'
    uint8_t mode;
    float yaw, yaw_vel, yaw_acc;
    float pitch, pitch_vel, pitch_acc;
    uint16_t crc16;
} VisionToGimbal_s;
#pragma pack()
```

#### 1.2 实现帧同步和验证函数

**文件**: `master_process.c` (在文件开头,包含头文件之后)

```c
/**
 * @brief 在缓冲区中查找'SP'帧头
 * @param buffer 缓冲区指针
 * @param len 缓冲区长度
 * @return 帧头位置索引,0xFF表示未找到
 */
static uint8_t FindSPHeader(uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len - 1; i++)
    {
        if (buffer[i] == 'S' && buffer[i+1] == 'P')
            return i;
    }
    return 0xFF; // 未找到
}

/**
 * @brief 验证四元数有效性
 * @param q 四元数数组 [w,x,y,z]
 * @return 1=有效, 0=无效
 */
static uint8_t ValidateQuaternion(float q[4])
{
    float norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    return (fabsf(norm_sq - 1.0f) < 0.01f); // 模在1.0的1%误差范围内
}

/**
 * @brief 验证视觉数据有效性
 * @param data VisionToGimbal数据包指针
 * @return 1=有效, 0=无效
 */
static uint8_t ValidateVisionData(VisionToGimbal_s *data)
{
    // 检查CRC
    uint16_t calc_crc = crc_16((uint8_t*)data, sizeof(VisionToGimbal_s) - 2);
    if (calc_crc != data->crc16)
        return 0;

    // 检查模式范围
    if (data->mode > 2)
        return 0;

    // 检查角度范围 (±π)
    if (fabsf(data->yaw) > 3.15f || fabsf(data->pitch) > 3.15f)
        return 0;

    return 1;
}
```

#### 1.3 替换接收函数 (DecodeVision)

**文件**: `master_process.c` (替换原有的DecodeVision函数)

**UART模式:**
```c
#ifdef VISION_USE_UART

static void DecodeVision()
{
    VisionToGimbal_s *packet = (VisionToGimbal_s *)vision_usart_instance->recv_buff;

    DaemonReload(vision_daemon_instance); // 喂狗

    // 验证数据有效性
    if (ValidateVisionData(packet))
    {
        // 映射模式
        switch (packet->mode)
        {
            case 0:
                recv_data.fire_mode = NO_FIRE;
                recv_data.target_state = NO_TARGET;
                break;
            case 1:
                recv_data.fire_mode = AUTO_AIM;
                recv_data.target_state = READY_TO_FIRE;
                break;
            case 2:
                recv_data.fire_mode = AUTO_FIRE;
                recv_data.target_state = READY_TO_FIRE;
                break;
        }

        // 复制角度数据 (弧度)
        recv_data.yaw = packet->yaw;
        recv_data.pitch = packet->pitch;

        // TODO: 如需使用速度/加速度数据,在这里提取
        // float yaw_vel = packet->yaw_vel;
        // float yaw_acc = packet->yaw_acc;

        recv_data.target_type = NO_TARGET_NUM; // 暂无目标类型
        recv_data.new_data = 1;
    }
}

#endif // VISION_USE_UART
```

**VCP模式 (类似修改):**
```c
#ifdef VISION_USE_VCP

static void DecodeVision(uint16_t recv_len)
{
    UNUSED(recv_len);
    VisionToGimbal_s *packet = (VisionToGimbal_s *)vis_recv_buff;

    DaemonReload(vision_daemon_instance); // 喂狗

    // 验证数据有效性
    if (ValidateVisionData(packet))
    {
        // (相同的映射逻辑...)
        switch (packet->mode)
        {
            case 0:
                recv_data.fire_mode = NO_FIRE;
                recv_data.target_state = NO_TARGET;
                break;
            case 1:
                recv_data.fire_mode = AUTO_AIM;
                recv_data.target_state = READY_TO_FIRE;
                break;
            case 2:
                recv_data.fire_mode = AUTO_FIRE;
                recv_data.target_state = READY_TO_FIRE;
                break;
        }

        recv_data.yaw = packet->yaw;
        recv_data.pitch = packet->pitch;
        recv_data.target_type = NO_TARGET_NUM;
        recv_data.new_data = 1;
    }
}

#endif // VISION_USE_VCP
```

#### 1.4 替换发送函数 (VisionSend)

**文件**: `master_process.c` (替换原有的VisionSend函数)

```c
void VisionSend()
{
    static GimbalToVision_s send_packet;

    // 填充帧头
    send_packet.head[0] = VISION_SP_HEADER_1;
    send_packet.head[1] = VISION_SP_HEADER_2;

    // 映射工作模式 (0→1, 1→2, 2→3)
    switch (send_data.work_mode)
    {
        case VISION_MODE_AIM:       // 0
            send_packet.mode = 1;
            break;
        case VISION_MODE_SMALL_BUFF: // 1
            send_packet.mode = 2;
            break;
        case VISION_MODE_BIG_BUFF:  // 2
            send_packet.mode = 3;
            break;
        default:
            send_packet.mode = 0;    // IDLE
            break;
    }

    // 获取四元数数据 (需要在阶段2实现INS_GetQuaternion)
    float *q = INS_GetQuaternion();
    memcpy(send_packet.q, q, sizeof(float) * 4);

    // 验证四元数
    if (!ValidateQuaternion(send_packet.q))
    {
        LOGWARNING("[vision] Invalid quaternion, skipping send");
        return;
    }

    // 转换角度 (度 → 弧度)
    send_packet.yaw = send_data.yaw * VISION_DEG_TO_RAD;
    send_packet.pitch = send_data.pitch * VISION_DEG_TO_RAD;

    // 获取角速度 (需要在阶段2实现INS_GetGyro)
    float gyro[3];
    INS_GetGyro(gyro);
    send_packet.yaw_vel = gyro[2];    // Z轴角速度
    send_packet.pitch_vel = gyro[0];  // X轴角速度

    // 弹速和弹丸数
    send_packet.bullet_speed = send_data.bullet_speed;

    // 从裁判系统获取弹丸数 (需要包含referee相关头文件)
    // extern referee_info_t referee_data;
    // send_packet.bullet_count = referee_data.PowerHeatData.shooter_id1_17mm_cooling_heat;
    send_packet.bullet_count = 0; // 暂时使用0,后续补充

    // 计算CRC16
    send_packet.crc16 = crc_16((uint8_t*)&send_packet, sizeof(GimbalToVision_s) - 2);

    // 发送
#ifdef VISION_USE_VCP
    USBTransmit((uint8_t*)&send_packet, sizeof(GimbalToVision_s));
#endif
#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance, (uint8_t*)&send_packet,
              sizeof(GimbalToVision_s), USART_TRANSFER_DMA);
#endif
}
```

---

### 阶段2: 四元数和角速度数据访问

**目标**: 暴露INS模块的四元数和角速度数据供视觉通信使用

**修改文件:**
- `modules/imu/ins_task.h`
- `modules/imu/ins_task.c`

#### 2.1 添加访问函数声明

**文件**: `modules/imu/ins_task.h` (在文件末尾,`#endif`之前)

```c
/**
 * @brief 获取当前四元数估计值
 * @return 四元数数组指针 [w,x,y,z]
 */
float* INS_GetQuaternion(void);

/**
 * @brief 获取当前角速度
 * @param gyro 输出数组 [gyro_x, gyro_y, gyro_z] (rad/s)
 */
void INS_GetGyro(float gyro[3]);
```

#### 2.2 实现访问函数

**文件**: `modules/imu/ins_task.c` (在文件末尾添加)

```c
/**
 * @brief 获取当前四元数估计值
 * @return 四元数数组指针 [w,x,y,z]
 */
float* INS_GetQuaternion(void)
{
    return INS.q;
}

/**
 * @brief 获取当前角速度
 * @param gyro 输出数组 [gyro_x, gyro_y, gyro_z] (rad/s)
 */
void INS_GetGyro(float gyro[3])
{
    gyro[0] = INS.Gyro[0];
    gyro[1] = INS.Gyro[1];
    gyro[2] = INS.Gyro[2];
}
```

---

### 阶段3: 控制流程集成

**目标**: 在机器人命令层处理视觉控制逻辑

**修改文件:**
- `application/cmd/robot_cmd.c`
- `application/robot_def.h`

#### 3.1 添加转换常量

**文件**: `application/robot_def.h` (在文件末尾,`#endif`之前)

```c
// ==================== 视觉通信常量 ====================
#ifndef VISION_RAD_TO_DEG
#define VISION_RAD_TO_DEG (57.295779513f)  // 弧度转角度
#endif

#ifndef VISION_DEG_TO_RAD
#define VISION_DEG_TO_RAD (0.01745329251f) // 角度转弧度
#endif

// 视觉控制增益 (可调)
#define VISION_YAW_GAIN   (1.0f)
#define VISION_PITCH_GAIN (1.0f)
```

#### 3.2 实现视觉控制逻辑

**文件**: `application/cmd/robot_cmd.c` (在RobotCMDTask函数中,处理视觉数据的位置)

找到类似这样的代码块并修改:
```c
// 原有的视觉处理代码可能类似:
// if (switch_is_mid(rc_data[TEMP].rc.switch_left)) { ... }

// 替换为新的视觉控制逻辑:
/* 视觉控制模式 */
if (vision_recv_data->new_data)
{
    vision_recv_data->new_data = 0; // 清除标志

    switch (vision_recv_data->fire_mode)
    {
        case NO_FIRE:
            // 不控制,保持当前状态
            break;

        case AUTO_AIM:
            // 仅控制云台,不开火
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

            // 角度转换: 弧度 → 度
            gimbal_cmd_send.yaw = vision_recv_data->yaw * VISION_RAD_TO_DEG;
            gimbal_cmd_send.pitch = vision_recv_data->pitch * VISION_RAD_TO_DEG;

            // TODO: 如需使用速度前馈,在这里添加
            break;

        case AUTO_FIRE:
            // 控制云台并开火
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            gimbal_cmd_send.yaw = vision_recv_data->yaw * VISION_RAD_TO_DEG;
            gimbal_cmd_send.pitch = vision_recv_data->pitch * VISION_RAD_TO_DEG;

            // 开启摩擦轮和拨盘
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            break;
    }
}
```

---

### 阶段4: 配置USB虚拟串口

**目标**: 确保USB VCP模式正确配置

#### 4.1 检查编译宏

**文件**: `application/robot_def.h`

确保定义了:
```c
// 使用USB虚拟串口模式
#define VISION_USE_VCP

// 如果定义了UART模式,注释掉
// #define VISION_USE_UART
```

#### 4.2 验证USB初始化

检查 `Src/main.c` 或 `robot.c` 中是否正确调用了USB初始化。通常在CubeMX生成的代码中会自动调用 `MX_USB_DEVICE_Init()`。

---

## 验证策略

### 阶段0: CRC兼容性验证 (CRITICAL!)

**步骤:**
1. 在控制端和视觉端分别编写测试代码
2. 使用相同的测试数据计算CRC16
3. 比较结果,必须完全一致

**测试数据示例:**
```c
uint8_t test_packet[] = {
    'S', 'P', 0x01,              // 帧头 + mode
    0x00, 0x00, 0x80, 0x3F,      // q[0] = 1.0f
    0x00, 0x00, 0x00, 0x00,      // q[1] = 0.0f
    0x00, 0x00, 0x00, 0x00,      // q[2] = 0.0f
    0x00, 0x00, 0x00, 0x00,      // q[3] = 0.0f
    // ... 其余字段
};
```

**如果CRC不一致:** 必须统一算法后再继续!

### 单元测试

#### 测试1: 结构体大小验证

**验证内容:**
- [ ] `sizeof(GimbalToVision_s) == 44`
- [ ] `sizeof(VisionToGimbal_s) == 29`
- [ ] `VISION_SEND_SIZE == 44`
- [ ] `VISION_RECV_SIZE == 29`

**测试代码:**
```c
void TestStructSizes(void)
{
    LOGINFO("GimbalToVision size: %d (expected 44)", sizeof(GimbalToVision_s));
    LOGINFO("VisionToGimbal size: %d (expected 29)", sizeof(VisionToGimbal_s));
    LOGINFO("VISION_SEND_SIZE: %d", VISION_SEND_SIZE);
    LOGINFO("VISION_RECV_SIZE: %d", VISION_RECV_SIZE);
}
```

#### 测试2: 四元数验证

**验证内容:**
- [ ] 四元数模接近1.0 (误差<1%)
- [ ] 四元数顺序正确 (wxyz)

**测试方法:**
```c
float *q = INS_GetQuaternion();
float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
LOGINFO("Quaternion: [%.3f, %.3f, %.3f, %.3f], norm=%.3f",
        q[0], q[1], q[2], q[3], norm);
// 预期: norm ≈ 1.0
```

#### 测试3: 模式映射验证

**验证内容:**
- [ ] VISION_MODE_AIM (0) → mode=1
- [ ] VISION_MODE_SMALL_BUFF (1) → mode=2
- [ ] VISION_MODE_BIG_BUFF (2) → mode=3

### 集成测试

#### 测试4: 通信回环

**步骤:**
1. 编译并烧录控制端固件
2. USB连接C板和上位机
3. 在上位机监听数据:
   ```bash
   ls -l /dev/ttyACM*
   sudo minicom -D /dev/ttyACM0 -b 921600
   ```
4. 检查数据包格式

**预期结果:**
- 每秒约20个数据包
- 帧头 `0x53 0x50` ('SP')
- 数据包大小44字节

#### 测试5: 控制响应

**步骤:**
1. 视觉端发送控制指令
2. 观察云台响应
3. 验证角度转换正确性

**预期结果:**
- 云台平滑运动
- 无抖动
- 角度误差<0.5度

---

## 关键文件清单

### 必须修改的文件

| 文件路径 | 修改内容 | 优先级 | 说明 |
|---------|---------|-------|------|
| `modules/master_machine/master_process.h` | 更新缓冲区大小宏,添加SP协议结构体 | P0 | 必须先更新宏定义! |
| `modules/master_machine/master_process.c` | 实现SP协议收发逻辑 | P0 | 添加必要头文件 |
| `modules/imu/ins_task.h` | 添加访问函数声明 | P0 | - |
| `modules/imu/ins_task.c` | 实现访问函数 | P0 | - |
| `application/cmd/robot_cmd.c` | 集成视觉控制逻辑 | P0 | - |
| `application/robot_def.h` | 添加转换常量 | P0 | - |

### 参考文件 (只读)

| 文件路径 | 用途 |
|---------|------|
| `nyush-rm-vision/io/gimbal/gimbal.hpp` | SP协议定义参考 |
| `nyush-rm-vision/io/gimbal/gimbal.cpp` | SP协议实现参考 |
| `nyush-rm-vision/tools/crc.cpp` | CRC16算法参考 |
| `modules/algorithm/crc16.c` | 控制端CRC16实现 |
| `bsp/usb/bsp_usb.c` | USB VCP底层驱动 |

---

## 补充说明

### 裁判系统数据获取

如需使用裁判系统的弹丸数据,需要:

1. **包含头文件:**
   ```c
   #include "referee_task.h"
   ```

2. **访问全局数据:**
   ```c
   extern referee_info_t referee_data;

   // 在VisionSend中:
   send_packet.bullet_count = referee_data.PowerHeatData.shooter_id1_17mm_cooling_heat;
   ```

3. **注意事项:**
   - 裁判系统可能离线,需检查数据有效性
   - 不同机器人类型字段可能不同
   - 根据实际枪口选择正确字段

### 调试输出建议

添加调试宏便于开发:
```c
// 在 robot_def.h 中
#define VISION_DEBUG_LEVEL 2 // 0=关闭, 1=错误, 2=警告, 3=信息, 4=详细

#if VISION_DEBUG_LEVEL >= 3
#define VISION_INFO(...) LOGINFO(__VA_ARGS__)
#else
#define VISION_INFO(...)
#endif

// 使用:
VISION_INFO("[vision] Send: mode=%d, q=[%.3f,%.3f,%.3f,%.3f]",
            send_packet.mode, send_packet.q[0], send_packet.q[1],
            send_packet.q[2], send_packet.q[3]);
```

---

## 实施时间线

### 第1天: 准备和验证
- [ ] 更新宏定义 (VISION_RECV/SEND_SIZE)
- [ ] 验证CRC16兼容性 (CRITICAL!)
- [ ] 添加必要头文件

### 第2天: 协议层实现
- [ ] 添加SP协议结构体
- [ ] 实现验证函数
- [ ] 替换DecodeVision和VisionSend
- [ ] 编译测试

### 第3天: 数据访问层
- [ ] 添加INS访问函数
- [ ] 测试四元数有效性
- [ ] 验证角速度数据

### 第4天: 控制集成
- [ ] 实现视觉控制逻辑
- [ ] 测试模式切换
- [ ] 调试云台响应

### 第5-7天: 测试和调优
- [ ] 通信回环测试
- [ ] 协议兼容性测试
- [ ] 端到端自瞄测试
- [ ] 性能优化

---

## 关键发现

1. ✅ 控制系统已有四元数数据(`INS_t.q[4]`) - 但需通过函数访问
2. ✅ USB VCP缓冲区2048字节,足够容纳数据包
3. ⚠️ CRC16算法可能不兼容 - 必须验证!
4. ⚠️ 缓冲区大小宏定义必须更新
5. ✅ 工作模式映射逻辑已正确设计
6. ✅ USB通信方式清晰,无需额外硬件

---

## 风险缓解

| 风险 | 缓解措施 |
|-----|---------|
| **CRC不兼容** | 实施前验证,必要时统一算法 |
| **数据损坏** | 添加四元数验证,CRC校验 |
| **时序问题** | 监控延迟,超时检测,daemon机制 |
| **模式混淆** | 详细日志,安全互锁 |
| **缓冲区溢出** | 更新宏定义,验证结构体大小 |

---

## 性能优化建议

### 短期优化
- 当前发送频率: 20Hz
- **建议**: 提升到100Hz
- 修改 `robot_cmd.c` 中的 `VISION_SEND_DIV` 从10改为2

### 长期优化
- 使用速度前馈控制
- 实现卡尔曼滤波融合
- 添加时间戳同步

---

## 相关文档

- `control-integration-issues.md` - 详细的问题清单和修复建议
- `nyush-rm-vision/下位机通信协议文档.md` - SP协议原始文档

---

**文档维护**: NYUSH Robotics Club
**License**: MIT
**最后更新**: 2026-02-08

---

**⚠️ 再次提醒: 实施前必须验证CRC16兼容性!**
