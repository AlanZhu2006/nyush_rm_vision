# 标定指南（相机内参 + 手眼/Robot‑World 手眼）

本文档基于本仓库 `calibration/` 中的程序整理，覆盖相机内参标定、手眼标定，以及可选的 Robot‑World 手眼标定流程。

## 1. 目标与产出
1. 相机内参标定：得到 `camera_matrix` 与 `distort_coeffs`。
2. 手眼标定：得到 `R_camera2gimbal` 与 `t_camera2gimbal`（相机到云台）。
3. 可选 Robot‑World 手眼：输出标定板在世界坐标系的相对信息注释。

## 2. 准备
1. 标定板类型：对称圆点阵列，默认 `10 x 7`，圆心间距 `center_distance_mm`（毫米）。
2. 配置文件：`configs/calibration.yaml`。
3. 硬件连接：相机 + 云台板（C板 IMU/四元数），CAN/串口按配置可用。

`configs/calibration.yaml` 关键字段：
1. `pattern_cols`, `pattern_rows`, `center_distance_mm`
2. `R_gimbal2imubody`（IMU 机体系到云台体系的旋转，初值为单位阵）
3. 相机参数：`camera_name`, `exposure_ms`, `gain`, `vid_pid`
4. C板参数：`quaternion_canid`, `bullet_speed_canid`, `send_canid`, `can_interface`

## 3. 编译
```bash
just build
```

## 4. 步骤一：采集标定数据（图像 + 四元数）
运行采集程序 `capture`：
```bash
just calibrate capture --config=configs/calibration.yaml --output-folder=assets/img_with_q --cli-mode
```

操作说明：
1. 画面会显示 IMU 欧拉角 + 标定板识别结果。
2. 按 `s` 保存当前图像和四元数，按 `q` 退出。
3. 数据输出到 `assets/img_with_q/`，文件名为 `1.jpg/1.txt` 递增。
4. 四元数输出顺序为 `w x y z`（程序会提示）。

采集建议：
1. 覆盖多角度、多距离、不同视场位置。
2. 避免同一姿态重复采集。
3. IMU 欧拉角方向异常时先检查 `R_gimbal2imubody`。

## 5. 步骤二：相机内参标定
运行 `calibrate_camera`：
```bash
just calibrate camera --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

交互说明：
1. 每张图会显示识别效果，按任意键继续。
2. 控制台输出识别成功/失败。
3. 结束后打印 YAML 格式结果（包含重投影误差）。

将输出中的以下字段写入你的目标配置文件（例如 `configs/standard4.yaml`）：
1. `camera_matrix`
2. `distort_coeffs`

## 6. 步骤三：手眼标定（相机到云台）
运行 `calibrate_handeye`：
```bash
just calibrate handeye --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

交互说明：
1. 每张图会显示识别结果和云台欧拉角，用于检查 `R_gimbal2imubody` 是否正确。
2. 输出 YAML：`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`。
3. 同时输出“相机相对理想姿态的偏角”注释，便于快速检查安装姿态。

将以下字段写入你的目标配置文件：
1. `R_camera2gimbal`
2. `t_camera2gimbal`

## 7. 可选：Robot‑World 手眼标定
如需估计标定板在世界坐标系中的相对信息，运行：
```bash
just calibrate robotworld-handeye --config=configs/calibration.yaml --input-folder=assets/img_with_q
```

输出包含：
1. `R_camera2gimbal`、`t_camera2gimbal`
2. 注释：`标定板到世界坐标系原点的水平距离`
3. 注释：`标定板同竖直摆放时的偏角`

示例注释格式可参考 `configs/example.yaml`、`configs/standard3.yaml` 等。

## 8. 常见问题排查
1. 标定板识别失败：
   - 确认标定板为“对称圆点阵列”
   - 确认 `pattern_cols/rows` 与实际一致
   - 调整曝光、增益或光照
2. IMU 欧拉角方向异常：
   - 优先检查 `R_gimbal2imubody` 是否正确
3. 重投影误差偏大：
   - 增加姿态多样性和采样数量
   - 避免全部样本在同一距离或同一平面
4. 手眼结果不稳定：
   - 增加云台姿态变化范围
   - 提高标定板在视野内的覆盖范围
