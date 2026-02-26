# 自瞄使用说明（从标定到实车）

本文给出一套可执行流程：

1. 完成相机内参与手眼标定
2. 验证上下位机协议与方向一致性
3. 上车运行自瞄并进入可调试状态

适用仓库：`nyush-rm-control`（下位机） + `sp_vision_25`（上位机）。


## 1. 上车前准备

## 1.1 下位机（nyush-rm-control）

1. 编译（默认步兵）：

```bash
cd /home/nyu/Codespace/nyush-rm-control
just build robot=infantry
```

2. 烧录（按你们常用方式）：

```bash
just flash dfu
```

3. 关键确认：

- 视觉接口宏已启用：`application/robot_configs/robot_infantry.h` 中 `VISION_USE_VCP`
- 视觉接收串口初始化存在：`application/cmd/robot_cmd.c` 中 `VisionInit(&huart1)`


## 1.2 上位机（sp_vision_25）

1. 编译：

```bash
cd /home/nyu/Codespace/sp_vision_25
just build
```

2. 串口固定名（推荐）：

- 按 `readme.md` 的 udev 步骤将设备固定到 `/dev/gimbal`
- 在配置文件（如 `configs/standard3.yaml`）中确认：`com_port: "/dev/gimbal"`


## 2. 协议与链路自检（先做）

在 `sp_vision_25` 目录执行：

```bash
python3 tests/protocol_link_audit.py
```

这一步会做随机化协议一致性验证（CRC、长度、大小端、单位映射）并输出静态风险告警。

再做在线链路测试（有硬件时）：

```bash
just test imu
```

建议观察：

- 接收频率是否稳定（目标 > 50 Hz）
- mode、yaw/pitch、bullet_speed 是否连续
- 无连续 CRC 错误


## 3. 标定流程（必须）

## 3.1 采集标定数据

```bash
just calibrate capture --config=configs/calibration.yaml --output-folder=assets/img_with_q --cli-mode
```

采集要点：

1. 标定板覆盖多距离、多姿态、多视野位置
2. 每次姿态变化明显，避免重复样本
3. `s` 保存，`q` 退出


## 3.2 相机内参标定

```bash
just calibrate camera --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

将输出中的：

- `camera_matrix`
- `distort_coeffs`

写入目标配置（例如 `configs/standard3.yaml`）。


## 3.3 手眼标定（相机到云台）

```bash
just calibrate handeye --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

将输出中的：

- `R_camera2gimbal`
- `t_camera2gimbal`

写入目标配置。


## 3.4（可选）Robot-World 手眼

```bash
just calibrate robotworld-handeye --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

用于获得更完整的场地/姿态解释信息，非必须。


## 4. 方向一致性验证（强烈建议）

标定完成后，不要直接上实战，先做方向验证。

## 4.1 姿态回传方向验证

运行：

```bash
just test gimbal
```

缓慢手动转动云台，确认：

1. `state.yaw` 与你对“左/右转”的定义一致
2. `state.pitch` 与“抬头/低头”定义一致
3. 四元数解出的欧拉角与 `state` 同趋势


## 4.2 指令方向验证

在安全条件下发送小角度命令（低速、小幅）：

```bash
just run response-yaw --amp=3 --signal=triangle_wave
```

观察 `cmd_yaw` 与 `gimbal_yaw` 是否同向跟随；`pitch` 同理。


## 5. 自瞄主程序运行（实车）

推荐入口：

```bash
just run mt
```

原因：

- `mt_standard` 包含 `Shooter` 决策链，链路完整
- `standard` 入口已接入 `Shooter` 与 mode 门控，可用于单线程调试


## 6. 下位机操作要点（按当前代码语义）

根据 `nyush-rm-control/application/cmd/robot_cmd.c` 当前逻辑：

1. 左拨杆中位时，视觉指令会覆盖云台角度（`switch_left == mid`）
2. 自动开火还受 shoot 状态机影响（`shoot_mode`/摩擦轮/拨盘）
3. 推荐先在“只跟随不开火”验证通过后再启用自动开火


## 7. 常用调参顺序

优先顺序建议：

1. `yaw_offset`, `pitch_offset`（先把静态零偏消掉）
2. `low_speed_delay_time`, `high_speed_delay_time`（补偿总时延）
3. `first_tolerance`, `second_tolerance`, `judge_distance`（开火窗口）

调参原则：

- 先保证“跟得上”，再追求“打得准”
- 先低速目标，再高速目标
- 每次只改 1~2 个参数，记录前后命中和误差曲线


## 8. 故障排查速查表

1. **云台反向追踪**
   - 先查标定是否正确（`R_camera2gimbal`）
   - 再查方向映射宏是否匹配当前车（`GYRO2GIMBAL_DIR_YAW/PITCH/ROLL`）

2. **能跟随但不开火**
   - 确认运行的是 `mt_standard` 或修复后的 `standard`
   - 确认 `auto_fire: true`
   - 检查下位机 shoot 模式是否允许发射

3. **模式显示异常/不切换**
   - 检查下位机发给上位机的 `mode`
   - 检查 VCP 串口与设备名绑定

4. **程序不能自启**
   - 现在 `autostart.sh` 自带 fallback；若自启失败，优先检查可执行程序路径与权限

5. **云台在 ±180° 附近突跳**
   - 确认下位机已包含 `UnwrapVisionYawToImuTotal` 版本
   - 复测三角波 yaw 指令，观察是否仍存在整圈跳变


## 9. 最终验收标准（建议）

上车前，至少满足：

1. `tests/protocol_link_audit.py` 通过
2. `imu_communication_test` 频率稳定，无持续错误
3. 小角度指令下 yaw/pitch 同向响应
4. 静止目标连续跟随稳定（无明显反复横跳）
5. 移动目标下开火窗口与命中趋势可重复


---

如果你们后续要把这套流程变成“赛前一键检查”，建议把第 2、4、9 节封装成一个巡检脚本（协议检查 + 在线方向检查 + 参数完整性检查）。
