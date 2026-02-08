# NYUSH RM Vision 海康威视摄像头部署指南

本文档详细说明如何从标定开始，完整部署基于海康威视摄像头的视觉系统。

## 目录
- [前期准备](#前期准备)
- [第一步：相机内参标定](#第一步相机内参标定)
- [第二步：手眼标定](#第二步手眼标定)
- [第三步：配置参数文件](#第三步配置参数文件)
- [第四步：测试验证](#第四步测试验证)
- [第五步：部署运行](#第五步部署运行)
- [常见问题](#常见问题)

---

## 前期准备

### 1. 硬件准备
- 海康威视工业相机（如 MV-CS016-10UC）
- 标定板：**10列7行对称圆点图案**，中心距 **40mm**
  - ⚠️ 如果你的标定板规格不同，需要修改 `configs/calibration.yaml` 中的参数
- RoboMaster 开发板（C板或其他带IMU的下位机）
- USB线材（相机连接）、CAN或串口线（下位机通信）

### 2. 软件环境确认
确认你已经安装了以下依赖（根据 readme.md 第3.2节）：
```bash
# 检查海康 SDK 是否已安装
ls /opt/MVS/lib/64/ | grep libMvCameraControl.so

# 检查 OpenCV
pkg-config --modversion opencv4

# 检查其他依赖
dpkg -l | grep -E "libfmt-dev|libeigen3-dev|libspdlog-dev|libyaml-cpp-dev"
```

### 3. 编译项目
```bash
cd ~/Codespace/nyush-rm-vision
cmake -B build
make -C build/ -j$(nproc)
```

编译成功后，会在 `build/` 目录下生成各种可执行文件。

---

## 第一步：相机内参标定

相机内参标定用于获取相机的**内参矩阵**和**畸变系数**，这是后续所有视觉算法的基础。

### 1.1 配置标定参数文件

编辑 `configs/calibration.yaml`，确保以下参数正确：

```yaml
# 标定板参数（根据你的实际标定板修改）
pattern_cols: 10              # 标定板列数
pattern_rows: 7               # 标定板行数
center_distance_mm: 40        # 圆点中心距，单位：毫米

# 海康相机参数
camera_name: "hikrobot"
exposure_ms: 3                # 曝光时间，建议3-5ms
gain: 10.0                    # 增益
vid_pid: "2bdf:0001"          # 海康相机的VID:PID，根据实际情况修改

# 下位机参数（用于获取姿态数据）
quaternion_canid: 0x100       # 四元数CAN ID
bullet_speed_canid: 0x101     # 子弹速度CAN ID
send_canid: 0xff              # 发送CAN ID
can_interface: "can0"         # CAN接口名称

# 云台到IMU坐标系的旋转矩阵（先用单位矩阵）
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

### 1.2 获取相机VID和PID

如果不确定相机的VID:PID，使用以下命令查看：

```bash
# 插入相机后执行
lsusb | grep -i hik

# 或者查看详细信息
udevadm info -a -n /dev/video0 | grep -E 'idVendor|idProduct'
```

示例输出：
```
idVendor=="2bdf"
idProduct=="0001"
```

则 `vid_pid` 应填写为 `"2bdf:0001"`。

### 1.3 采集标定图片

运行标定图片采集程序：

```bash
./build/capture configs/calibration.yaml
```

程序说明：
- 窗口会显示实时图像和IMU的欧拉角（Z、Y、X）
- 程序会自动检测标定板的圆点
- **绿色线条**表示成功识别标定板

采集要求：
1. **准备20-30张**不同角度、不同位置的标定板图片
2. 覆盖图像的**各个区域**（左、右、上、下、中心）
3. 包含**不同倾斜角度**（俯视、仰视、侧视）
4. 确保标定板在图像中**完整可见**、**光照均匀**

操作步骤：
1. 移动标定板或相机到不同位置和角度
2. 当看到标定板被正确识别（绿色线条）时，按 **'s'** 保存
3. 重复上述步骤，采集足够数量的图片
4. 完成后按 **'q'** 退出

图片和对应的四元数文件会保存在 `assets/img_with_q/` 目录下：
```
assets/img_with_q/
├── 1.jpg
├── 1.txt
├── 2.jpg
├── 2.txt
...
```

### 1.4 执行相机内参标定

运行标定程序：

```bash
./build/calibrate_camera assets/img_with_q
```

程序会：
1. 逐个显示采集的图片和识别结果
2. 按任意键继续下一张
3. 最后输出标定结果

标定结果示例：
```yaml
# 重投影误差: 0.3145px
camera_matrix: [1851.707, 0, 721.126, 0, 1851.818, 571.699, 0, 0, 1]
distort_coeffs: [-0.0937, 0.1895, -0.0004, -0.0041, 0]
```

**重要提示**：
- 重投影误差应小于 **0.5像素**，否则需要重新采集图片
- 保存好这个结果，后续需要填入配置文件

---

## 第二步：手眼标定

手眼标定用于获取**相机坐标系相对于云台坐标系的变换关系**。

### 2.1 确定 R_gimbal2imubody

这是云台坐标系到IMU坐标系的旋转矩阵，需要根据实际安装方式确定。

**步骤**：
1. 在采集图片时，程序显示的 Z、Y、X 角度分别对应IMU坐标系的：
   - Z：绕Z轴旋转（yaw）
   - Y：绕Y轴旋转（pitch）
   - X：绕X轴旋转（roll）

2. 固定云台不动，观察这些角度值：
   - 云台向左转动（yaw+），对应哪个轴正向？
   - 云台向上抬升（pitch+），对应哪个轴正向？
   - 判断坐标系的对应关系

3. 常见情况：
   ```yaml
   # IMU坐标系与云台坐标系重合
   R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

   # IMU绕某轴旋转90度安装
   # 具体需要根据实际情况调整
   ```

### 2.2 更新配置并重新采集

将上一步得到的 `camera_matrix` 和 `distort_coeffs` 填入 `configs/calibration.yaml`：

```yaml
# 相机内参（从第一步获得）
camera_matrix: [1851.707, 0, 721.126, 0, 1851.818, 571.699, 0, 0, 1]
distort_coeffs: [-0.0937, 0.1895, -0.0004, -0.0041, 0]

# 云台到IMU的旋转矩阵（根据实际情况填写）
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

**重新采集图片**用于手眼标定：

```bash
# 清空之前的图片（如果需要）
rm -rf assets/img_with_q/*

# 重新采集
./build/capture configs/calibration.yaml
```

手眼标定采集要求：
- 采集 **15-25张** 图片
- 需要**改变云台姿态**（yaw和pitch都要变化）
- 标定板位置可以固定在地面或墙上
- 确保每张图片的云台姿态都不同

### 2.3 执行手眼标定

运行手眼标定程序：

```bash
./build/calibrate_handeye assets/img_with_q
```

程序会：
1. 显示每张图片的云台欧拉角（yaw、pitch、roll）
2. 检查 `R_gimbal2imubody` 是否正确
3. 输出手眼标定结果

标定结果示例：
```yaml
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机同理想情况的偏角: yaw-1.61 pitch-0.82 roll-0.61 degree
R_camera2gimbal: [0.0282, -0.0141, 0.9995, -0.9995, -0.0110, 0.0281, 0.0106, -0.9998, -0.0144]
t_camera2gimbal: [0.0455, 0.1054, 0.0346]
```

**验证**：
- 偏角应该接近实际安装误差（通常小于5度）
- `t_camera2gimbal` 是相机到云台中心的平移，单位米
- 如果偏角过大，检查 `R_gimbal2imubody` 是否正确

---

## 第三步：配置参数文件

为你的机器人创建专属配置文件（或修改现有配置）。

### 3.1 创建配置文件

```bash
# 复制模板
cp configs/standard4.yaml configs/my_robot.yaml
```

### 3.2 填写标定结果

编辑 `configs/my_robot.yaml`，填入标定得到的参数：

```yaml
#####-----相机参数-----#####
camera_name: "hikrobot"
exposure_ms: 2                # 运行时曝光，可以调整（1-5ms）
gain: 16                      # 运行时增益，根据光照调整
vid_pid: "2bdf:0001"          # 你的相机VID:PID

#####-----相机标定结果-----#####
# 重投影误差: 0.3145px
camera_matrix: [1851.707, 0, 721.126, 0, 1851.818, 571.699, 0, 0, 1]
distort_coeffs: [-0.0937, 0.1895, -0.0004, -0.0041, 0]

#####-----手眼标定结果-----#####
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机同理想情况的偏角: yaw-1.61 pitch-0.82 roll-0.61 degree
R_camera2gimbal: [0.0282, -0.0141, 0.9995, -0.9995, -0.0110, 0.0281, 0.0106, -0.9998, -0.0144]
t_camera2gimbal: [0.0455, 0.1054, 0.0346]

#####-----下位机通信参数-----#####
# 如果使用CAN通信
quaternion_canid: 0x100       # 根据下位机协议填写
bullet_speed_canid: 0x101
send_canid: 0xff
can_interface: "can0"         # can0 or can1

# 如果使用串口通信（二选一）
com_port: "/dev/gimbal"       # 需要先配置udev规则
```

### 3.3 配置串口（如果使用串口通信）

海康相机使用USB连接，下位机可以通过串口通信。参考 readme.md 第3.2节第7点：

```bash
# 1. 连接下位机，查找串口设备
ls /dev/ttyACM*

# 2. 获取设备ID
udevadm info -a -n /dev/ttyACM0 | grep -E 'serial|idVendor|idProduct'

# 示例输出
ATTRS{idVendor}=="1234"
ATTRS{idProduct}=="5678"
ATTRS{serial}=="A1234567"

# 3. 创建udev规则
sudo nano /etc/udev/rules.d/99-usb-serial.rules

# 4. 写入以下内容（替换为实际ID）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", ATTRS{serial}=="A1234567", SYMLINK+="gimbal"

# 5. 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 6. 授予权限
sudo usermod -a -G dialout $USER

# 7. 重新登录后检查
ls -l /dev/gimbal
```

### 3.4 配置CAN（如果使用CAN通信）

```bash
# 1. 创建udev规则自动启动CAN
sudo nano /etc/udev/rules.d/99-can-up.rules

# 2. 写入以下内容
ACTION=="add", KERNEL=="can0", RUN+="/sbin/ip link set can0 up type can bitrate 1000000"
ACTION=="add", KERNEL=="can1", RUN+="/sbin/ip link set can1 up type can bitrate 1000000"

# 3. 手动启动CAN（测试用）
sudo ip link set can0 up type can bitrate 1000000

# 4. 检查CAN状态
ip -details link show can0
```

### 3.5 调整算法参数

根据实际需求调整算法参数：

```yaml
#####-----自瞄参数-----#####
enemy_color: "red"            # 或 "blue"

min_confidence: 0.8           # 识别置信度阈值
yaw_offset: -2                # yaw偏移补偿，degree
pitch_offset: 0               # pitch偏移补偿，degree

#####-----射击参数-----#####
first_tolerance: 3            # 近距离射击容差，degree
second_tolerance: 2           # 远距离射击容差，degree
judge_distance: 2             # 距离判断阈值，meter
auto_fire: true               # 是否自动开火

#####-----轨迹规划器参数-----#####
fire_thresh: 0.003            # 开火阈值
max_yaw_acc: 50               # 最大yaw加速度
max_pitch_acc: 100            # 最大pitch加速度
```

---

## 第四步：测试验证

在正式部署前，需要进行各模块的单独测试。

### 4.1 相机测试

```bash
# 测试相机能否正常打开和采图
./build/camera_test configs/my_robot.yaml
```

预期结果：
- 能看到实时图像窗口
- 图像清晰，无明显畸变
- 按 'q' 退出

### 4.2 下位机通信测试

如果使用CAN：
```bash
./build/cboard_test configs/my_robot.yaml
```

如果使用串口：
```bash
./build/gimbal_test configs/my_robot.yaml
```

预期结果：
- 能够正常接收IMU四元数数据
- 数据刷新率正常（通常100-1000Hz）
- 云台转动时，yaw/pitch角度相应变化

### 4.3 识别器测试

```bash
# 工业相机识别测试
./build/camera_detect_test configs/my_robot.yaml
```

操作：
- 将摄像头对准装甲板目标
- 检查识别框是否准确
- 调整 `exposure_ms` 和 `gain` 参数以获得最佳效果

### 4.4 完整系统测试

```bash
# 运行最小视觉系统
./build/minimum_vision_system configs/my_robot.yaml
```

这是一个轻量级测试程序，验证：
- 相机、下位机同时工作
- 识别、跟踪、决策流程正常
- 数据通信正常

### 4.5 自瞄录制测试

```bash
# 运行自瞄并录制
./build/auto_aim_test configs/my_robot.yaml
```

这个程序会录制视频并保存调试数据，用于离线分析。

---

## 第五步：部署运行

### 5.1 选择运行程序

根据你的兵种选择对应的主程序：

```bash
# 步兵（标准版）
./build/standard configs/my_robot.yaml

# 步兵（MPC版本，使用轨迹规划）
./build/standard_mpc configs/my_robot.yaml

# 哨兵
./build/sentry configs/my_robot.yaml

# 无人机
./build/uav configs/my_robot.yaml

# 调试版本（带可视化和日志）
./build/mt_auto_aim_debug configs/my_robot.yaml
```

### 5.2 PlotJuggler 实时监控（可选）

安装 PlotJuggler：
```bash
sudo apt install ros-*-plotjuggler  # 如果安装了ROS
# 或者
sudo snap install plotjuggler
```

使用：
1. 程序运行时会将数据以 UDP 方式发送
2. PlotJuggler 监听对应端口即可实时绘制曲线
3. 查看 yaw/pitch 跟踪误差、目标速度、开火状态等

### 5.3 配置自启动

编辑 `autostart.sh`：

```bash
#!/bin/bash
sleep 5
cd ~/Codespace/nyush-rm-vision/  # 修改为你的实际路径
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./build/standard_mpc configs/my_robot.yaml"  # 选择你的程序
```

创建自启动服务：

```bash
# 1. 创建目录
mkdir -p ~/.config/autostart/

# 2. 创建 .desktop 文件
nano ~/.config/autostart/sp_vision.desktop

# 3. 写入以下内容（修改路径）
[Desktop Entry]
Type=Application
Exec=/home/你的用户名/Codespace/nyush-rm-vision/autostart.sh
Name=sp_vision

# 4. 授予执行权限
chmod +x autostart.sh
```

### 5.4 日志管理

程序日志保存在 `logs/` 目录：

```bash
# 查看最新日志
tail -f logs/*.screenlog

# 或者连接到运行的 screen 会话
screen -r
```

---

## 常见问题

### Q1: 相机无法打开
- 检查 USB 连接
- 检查 MVS SDK 是否正确安装
- 检查 `vid_pid` 参数是否正确
- 尝试运行海康官方的 MVS 软件测试相机

### Q2: 标定板无法识别
- 确保标定板规格与配置文件一致
- 调整曝光和增益，避免过曝或欠曝
- 确保标定板平整，圆点清晰
- 检查图案类型是否为"对称圆点"

### Q3: 重投影误差过大
- 增加采集图片数量（30张以上）
- 确保图片覆盖画面各个区域
- 重新拍摄模糊或过曝的图片
- 检查标定板是否变形

### Q4: 下位机通信失败
- CAN：检查波特率、CAN接口是否up、线路连接
- 串口：检查udev规则、权限、波特率
- 使用 `candump can0` 或串口调试工具验证

### Q5: 识别效果不好
- 调整曝光时间（减少运动模糊）
- 调整 `min_confidence` 阈值
- 检查神经网络模型路径是否正确
- 尝试切换 `use_traditional: true` 使用传统方法

### Q6: 自瞄打不准
- 检查手眼标定结果的偏角是否合理
- 调整 `yaw_offset` 和 `pitch_offset` 补偿
- 检查 `t_camera2gimbal` 平移向量是否正确
- 使用慢动作视频分析弹道

### Q7: 帧率过低
- 降低图像分辨率
- 使用 GPU 推理（需要安装对应驱动）
- 启用 ROI 裁剪减少处理区域
- 优化神经网络模型（量化、剪枝）

### Q8: 系统崩溃或重启
- 检查日志文件定位错误
- 验证配置文件格式是否正确
- 检查内存使用情况
- 使用调试版本程序排查问题

---

## 进阶优化

### 1. 弹道补偿优化
通过实际测试调整偏移量：
```yaml
yaw_offset: -2.5    # 根据实际命中点调整
pitch_offset: 1.0   # 根据不同距离分别调整
```

### 2. 动态参数调整
根据敌方运动状态自适应调整参数（需要修改代码）

### 3. 多相机方案
哨兵等兵种可能需要多个相机，修改相机初始化代码支持多设备

### 4. 性能优化
- 使用 Intel GPU 加速（参考 readme.md 第3.2节第6点）
- 启用异步推理
- 优化图像预处理流程

---

## 参考资料

- 项目 README: `readme.md`
- 标定程序源码: `calibration/` 目录
- 配置文件示例: `configs/standard4.yaml`
- 测试程序: `tests/` 目录

---

**祝你部署顺利！如有问题，请查阅项目 issues 或联系开发团队。**

最后更新：2026-02-08
