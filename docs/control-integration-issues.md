# 协议整合计划疏漏检查报告

检查日期: 2026-02-08
检查文档: `/home/pengyue/Codespace/nyush-rm-control/docs/vision-sp-protocol-integration.md`

---

## 🔴 严重问题 (P0 - 必须修复)

### 1. **工作模式枚举值映射错误** ⚠️

**问题描述:**
控制端的 `Work_Mode_e` 定义 (master_process.h:78-81):
```c
VISION_MODE_AIM = 0
VISION_MODE_SMALL_BUFF = 1
VISION_MODE_BIG_BUFF = 2
```

但视觉端期望的 mode 值 (gimbal.hpp:48-54):
```cpp
enum class GimbalMode {
    IDLE = 0,        // 空闲
    AUTO_AIM = 1,    // 自瞄
    SMALL_BUFF = 2,  // 小符
    BIG_BUFF = 3     // 大符
};
```

**计划中的映射 (错误!):**
```c
VISION_MODE_AIM → 1        // 应该是 0→1 的映射
VISION_MODE_SMALL_BUFF → 2 // 应该是 1→2 的映射
VISION_MODE_BIG_BUFF → 3   // 应该是 2→3 的映射
```

**正确的映射应该是:**
```c
// 控制端枚举值 → 视觉端期望值
switch (send_data.work_mode) {
    case VISION_MODE_AIM:        send_packet.mode = 1; break; // 0→1
    case VISION_MODE_SMALL_BUFF: send_packet.mode = 2; break; // 1→2
    case VISION_MODE_BIG_BUFF:   send_packet.mode = 3; break; // 2→3
    default:                     send_packet.mode = 0; break; // 其他→0 (IDLE)
}
```

**影响:** 如果不修复,视觉端会收到错误的工作模式,导致功能错乱!

---

### 2. **CRC16算法可能不兼容** ⚠️

**问题描述:**
- 控制端使用 `crc_16()`,多项式 `0xA001` (CRC-16-ANSI 反向多项式)
- 视觉端使用 `tools::get_crc16()`,但使用的 **CRC16_TABLE 与控制端不同**

**证据:**
- 控制端 CRC表第一行开头: `0x0000, 0xC0C1, 0xC181, ...` (未显示完整)
- 视觉端 CRC表第一行开头: `0x0000, 0x1189, 0x2312, 0x329b, ...`

**关键差异:** 表格完全不同,说明多项式不同!

**验证方法 (计划中缺失):**
```c
// 在控制端测试代码中
uint8_t test_data[] = {'S', 'P', 0x01, 0x00, 0x00, 0x00, 0x00}; // 简单测试包
uint16_t crc = crc_16(test_data, sizeof(test_data));
LOGINFO("Control CRC16: 0x%04X", crc);
// 同样数据在视觉端计算,比较结果
```

**修复方案:**
1. **选项A (推荐):** 在控制端使用视觉端的CRC16算法 (需要移植CRC表)
2. **选项B:** 在视觉端修改为控制端的CRC16算法 (需要修改视觉端代码)
3. **选项C:** 统一到标准CRC-16-CCITT (0x1021 多项式)

**计划中需要添加:** CRC兼容性测试和验证步骤

---

### 3. **接收数据缓冲区大小宏定义未更新** ⚠️

**问题描述:**
`master_process.h:7` 定义:
```c
#define VISION_RECV_SIZE 18u  // 旧SeaSky协议大小
```

但新的 `VisionToGimbal_s` 结构体大小是 **29 字节**!

**影响:**
- UART 模式下接收缓冲区太小,会导致数据丢失
- USB VCP 模式可能正常(缓冲区2048字节),但逻辑不一致

**修复:**
```c
#define VISION_RECV_SIZE 29u  // 新SP协议接收包大小
#define VISION_SEND_SIZE 44u  // 新SP协议发送包大小 (原来36u)
```

---

### 4. **INS数据访问方式未明确** ⚠️

**问题描述:**
- 计划中提到 `INS_t.q[4]` 可以直接访问
- 但实际代码中 `INS` 是 `ins_task.c` 中的 **static 变量**,其他文件无法访问!

**当前代码 (ins_task.c:24):**
```c
static INS_t INS;  // 静态变量,文件作用域
```

**计划中的方案是正确的:** 需要添加 `INS_GetQuaternion()` 函数
**但需要澄清:** 这不是"可选的优化",而是**必须的实现**!

**补充说明:**
- `INS_Init()` 返回的是 `attitude_t*`,不是 `INS_t*`
- `attitude_t` 不包含四元数,只有欧拉角
- 必须通过新增的访问函数来获取四元数

---

## 🟡 重要问题 (P1 - 强烈建议修复)

### 5. **IMU数据来源描述不准确**

**问题描述:**
计划中多处提到 `gimba_IMU_data` (拼写错误),实际应该是:
- 在 `robot_task.h` 的 INS task 中: `imu_data` (类型 `attitude_t*`)
- 在 message_center 传递的数据中: `gimbal_imu_data` (在 `Gimbal_Upload_Data_s` 中)

**正确的数据流:**
```
INS task (1000Hz)
  └─> imu_data = INS_Init()
  └─> imu_data->{Yaw,Pitch,Roll,Gyro}
  └─> VisionSetAltitude(imu_data->Yaw, imu_data->Pitch, imu_data->Roll)

Gimbal app
  └─> 通过 message_center 发布 gimbal_imu_data
  └─> robot_cmd 订阅并使用
```

**角速度数据来源:**
- 计划中说从 `gimba_IMU_data->Gyro[2]` 获取
- **正确来源:** `imu_data->Gyro[0/1/2]` (在 INS task 中)
- 或者从 `INS` 内部结构获取 (需要通过 `INS_GetGyro()` 函数)

**修复:** 更新文档中的变量名,明确数据流

---

### 6. **VCP模式离线回调无重启逻辑**

**问题描述:**
当前的 `VisionOfflineCallback()` (master_process.c:82-88):
```c
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance); // 仅UART模式重启
#endif
    LOGWARNING("[vision] vision offline, restart communication.");
}
```

**问题:** VCP模式下没有重启逻辑,只打印警告!

**影响:** USB通信故障后无法自动恢复

**修复方案:**
```c
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
#ifdef VISION_USE_VCP
    // USB VCP通常不需要重启,因为USB层会自动处理
    // 但可以添加计数器,多次离线后重新枚举USB设备
    static uint8_t offline_count = 0;
    offline_count++;
    if (offline_count > 10) {
        // 可选: 重新初始化USB (需要实现USBReinit())
        // USBReinit();
        offline_count = 0;
    }
#endif
    LOGWARNING("[vision] vision offline, restart communication.");
}
```

---

### 7. **帧同步逻辑未完整实现**

**问题描述:**
计划提供了 `FindSPHeader()` 函数,但未说明如何集成到接收流程中。

**当前SeaSky协议的处理方式:**
- 使用 `get_protocol_info()` 自动处理帧同步
- 有完整的状态机

**新SP协议需要:**
1. DMA循环接收到固定大小缓冲区
2. 帧头搜索和对齐
3. 完整包验证
4. 错位恢复机制

**计划中的 `DecodeVision()` 实现过于简化:**
```c
// 当前计划的实现
VisionToGimbal_s *packet = (VisionToGimbal_s *)vision_usart_instance->recv_buff;
if (ValidateVisionData(packet)) { ... }
```

**问题:**
- 假设数据已经对齐,没有处理帧错位
- 没有处理部分接收的情况
- 没有循环缓冲区机制

**建议补充:** 完整的帧同步状态机实现示例

---

### 8. **四元数顺序未验证**

**问题描述:**
计划中说四元数是 `wxyz` 顺序,但未验证 `INS_t.q[4]` 的实际顺序。

**不同库的约定:**
- Eigen (视觉端使用): `[w,x,y,z]` 或 `[x,y,z,w]` (取决于构造方式)
- 控制端 QuaternionEKF: 需要查看实际实现

**验证方法 (计划中缺失):**
```c
// 在已知姿态下 (如水平静止) 检查四元数
// 水平静止时应该接近 [1, 0, 0, 0] (w,x,y,z) 或 [0, 0, 0, 1] (x,y,z,w)
LOGINFO("Quaternion: [%.3f, %.3f, %.3f, %.3f]",
        INS.q[0], INS.q[1], INS.q[2], INS.q[3]);
```

**建议:** 添加四元数顺序验证步骤到测试计划

---

### 9. **裁判系统数据获取未说明**

**问题描述:**
`bullet_speed` 和 `bullet_count` 需要从裁判系统获取,但计划中只有 TODO,没有具体方法。

**实际获取方式 (需要补充到计划):**
```c
// 裁判系统数据在 referee 模块中
#include "referee_task.h"

// 在 VisionSend() 中:
extern referee_info_t referee_data; // 裁判系统全局数据

send_packet.bullet_speed = referee_data.GameRobotState.shooter_id1_17mm_speed_limit;
send_packet.bullet_count = referee_data.PowerHeatData.shooter_id1_17mm_cooling_heat; // 或其他字段
```

**注意事项:**
- 裁判系统可能离线,需要检查数据有效性
- 不同机器人类型的字段可能不同
- 需要根据实际使用的枪口选择正确的字段

---

## 🟢 次要问题 (P2 - 建议改进)

### 10. **代码示例缺少必要的头文件包含**

计划中的代码需要以下头文件,但未说明:
```c
#include "ins_task.h"      // INS_GetQuaternion()
#include <string.h>         // memcpy()
#include <math.h>           // fabsf()
#include "bsp_log.h"        // LOGINFO/LOGWARNING
#include "referee_task.h"   // 裁判系统数据
```

---

### 11. **VisionInit 函数签名在VCP模式下的参数问题**

**问题:**
```c
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
```
VCP模式下这个参数被标记为 `UNUSED`,但调用者仍需传递。

**改进建议:**
```c
// 选项1: 条件编译
#ifdef VISION_USE_UART
Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle);
#else
Vision_Recv_s *VisionInit(void);
#endif

// 选项2: 统一传NULL (当前方案,更简单)
Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle); // VCP模式传NULL
```

---

### 12. **角度单位混乱风险**

**问题:** 计划中多处涉及角度单位转换,容易出错:
- 控制端 IMU: 角度(度)
- 视觉端: 角度(弧度)
- 角速度: 弧度/秒

**建议:** 在代码中添加明确的单位注释和断言:
```c
// 明确单位
float yaw_deg = imu_data->Yaw;           // 度
float yaw_rad = yaw_deg * DEG_TO_RAD;    // 弧度

// 添加范围检查
if (fabsf(yaw_rad) > 2*M_PI) {
    LOGWARNING("Yaw angle out of range: %.2f rad", yaw_rad);
}
```

---

### 13. **测试策略过于抽象**

**问题:** 计划中的测试部分缺少:
- 具体的测试程序/脚本
- 逐步测试命令
- 预期输出示例

**建议补充:**
```markdown
### 测试1: CRC16兼容性验证

**步骤:**
1. 在控制端添加测试代码:
   ```c
   uint8_t test_packet[] = {
       'S', 'P', 0x01,
       0x00, 0x00, 0x80, 0x3F,  // q[0] = 1.0f
       // ... 其余字段
   };
   uint16_t crc = crc_16(test_packet, sizeof(test_packet)-2);
   LOGINFO("CRC16: 0x%04X", crc);
   ```
2. 在视觉端计算相同数据的CRC
3. 比较结果,必须完全一致

**预期结果:** 两端CRC值相同
```

---

### 14. **性能影响未量化**

**问题:** 新协议数据包更大,但未分析性能影响:
- 发送: 44字节 vs 原来 ~22字节 (翻倍)
- 接收: 29字节 vs 原来 18字节 (+61%)

**建议补充:**
```markdown
### 性能分析

**USB带宽使用:**
- 发送频率: 20Hz → 100Hz (建议优化)
- 发送数据量: 44字节 × 100Hz = 4.4KB/s = 35.2Kbps
- USB 2.0 全速: 12Mbps
- 带宽占用率: 0.29% ✓ 充足

**CPU开销:**
- CRC16计算: ~44字节 × 2 (收+发) = 88字节/次
- 在200MHz CPU上预计 < 10us
- 占用率: 可忽略

**结论:** 性能不是瓶颈
```

---

### 15. **向后兼容实现不完整**

**问题:** 计划提到可以用编译宏实现向后兼容,但未提供完整实现。

**建议补充:**
```c
// 在 master_process.c 中

#ifdef VISION_PROTOCOL_SP
// SP协议实现
static void DecodeVision() { /* SP解析 */ }
void VisionSend() { /* SP发送 */ }
#else
// SeaSky协议实现 (原代码)
static void DecodeVision() { /* SeaSky解析 */ }
void VisionSend() { /* SeaSky发送 */ }
#endif
```

---

## 📋 需要补充的内容

### 16. **完整的数据流图**

计划中缺少清晰的数据流图,建议添加:
```
[INS Task 1kHz]
    │
    ├─> attitude_t imu_data (Yaw/Pitch/Roll/Gyro)
    │   └─> VisionSetAltitude() 每50ms (20Hz)
    │
    └─> INS_t INS (q[4] 四元数)
        └─> INS_GetQuaternion() [新增函数]
            └─> VisionSend() 构建 GimbalToVision
                └─> USB/UART 发送 44字节

[USB/UART 接收] 29字节 VisionToGimbal
    │
    └─> DecodeVision() 回调
        └─> 解析+验证
            └─> Vision_Recv_s recv_data
                └─> robot_cmd.c 订阅
                    └─> 云台控制指令
```

---

### 17. **错误处理策略**

计划中缺少完整的错误处理说明:

**需要处理的错误情况:**
1. CRC校验失败 → 丢弃数据包,计数器+1,超过阈值触发告警
2. 四元数模不为1 → 跳过该次发送,记录错误
3. 帧头未找到 → 继续搜索,最多搜索缓冲区大小
4. 通信超时 → daemon触发离线回调
5. USB断开 → 自动重连机制
6. 数据范围异常 → 限幅或丢弃

---

### 18. **调试输出建议**

建议添加调试宏和日志等级控制:
```c
// 在 robot_def.h 中添加
#define VISION_DEBUG_LEVEL 2 // 0=关闭, 1=错误, 2=警告, 3=信息, 4=详细

#if VISION_DEBUG_LEVEL >= 4
#define VISION_DEBUG_VERBOSE(...) LOGINFO(__VA_ARGS__)
#else
#define VISION_DEBUG_VERBOSE(...)
#endif

// 使用示例
VISION_DEBUG_VERBOSE("[vision] Send: mode=%d, q=[%.3f,%.3f,%.3f,%.3f]",
                     send_packet.mode,
                     send_packet.q[0], send_packet.q[1],
                     send_packet.q[2], send_packet.q[3]);
```

---

## ✅ 计划中正确的部分

1. ✓ USB VCP 通信配置正确 (2048字节缓冲区足够)
2. ✓ 数据结构定义清晰 (GimbalToVision 44字节, VisionToGimbal 29字节)
3. ✓ 使用四元数的策略正确 (避免万向锁)
4. ✓ 分阶段实施策略合理
5. ✓ 识别了需要添加的访问函数
6. ✓ USB通信硬件连接说明清楚

---

## 🔧 优先修复建议

### 立即修复 (实施前必须解决):
1. **CRC16算法兼容性** - 可能导致所有通信失败
2. **工作模式映射** - 导致功能错乱
3. **接收缓冲区大小** - UART模式下会丢数据
4. **INS数据访问方式** - 编译错误

### 实施中修复:
5. IMU数据来源描述
6. VCP离线回调
7. 帧同步逻辑
8. 裁判系统数据获取

### 测试中验证:
9. 四元数顺序
10. 角度单位转换
11. 性能影响

---

## 📝 总结

**总体评价:** 计划的大框架是正确的,但存在多个关键技术细节的疏漏。

**风险等级:**
- 🔴 高风险: 4个 (CRC算法、模式映射、缓冲区大小、INS访问)
- 🟡 中风险: 5个
- 🟢 低风险: 9个

**建议:**
1. 优先验证CRC16算法兼容性 (最关键!)
2. 修正所有P0问题后再开始实施
3. 补充详细的测试步骤和预期结果
4. 添加更多的错误处理和调试输出

**预计额外工作量:** +2-3天 (修复疏漏 + 充分测试)
