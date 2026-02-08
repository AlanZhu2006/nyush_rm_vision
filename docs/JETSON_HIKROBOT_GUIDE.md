# Jetson 平台海康威视相机驱动安装配置指南

本文档详细说明如何在 NVIDIA Jetson 系列开发板（ARM64 架构）上安装和配置海康威视工业相机驱动及相关环境。

## 目录

- [平台说明](#平台说明)
- [前期准备](#前期准备)
- [第一步：安装海康威视 MVS SDK（ARM64版本）](#第一步安装海康威视-mvs-sdkarm64版本)
- [第二步：安装依赖项](#第二步安装依赖项)
- [第三步：编译项目](#第三步编译项目)
- [第四步：相机测试](#第四步相机测试)
- [第五步：性能优化](#第五步性能优化)
- [常见问题排查](#常见问题排查)
- [Jetson 特定注意事项](#jetson-特定注意事项)

---

## 平台说明

### 支持的 Jetson 平台

本指南适用于以下 Jetson 平台：

- **Jetson Orin** 系列（推荐）：Orin NX、Orin Nano、AGX Orin
- **Jetson Xavier** 系列：Xavier NX、AGX Xavier
- **Jetson Nano** 系列：Jetson Nano、Jetson Nano 2GB（性能受限）

### 推荐配置

- **硬件**：Jetson Orin NX 16GB 或更高
- **JetPack 版本**：JetPack 5.x 或 6.x（Ubuntu 20.04/22.04）
- **相机**：海康威视 MV-CS 系列 USB 工业相机
- **存储**：至少 32GB，推荐使用 NVMe SSD

### 架构说明

Jetson 系列开发板基于 ARM64（aarch64）架构，与传统 x86_64 架构不同。因此：

- 必须使用 **ARM64 专用版本** 的海康威视 SDK
- 某些依赖库需要从源码编译或使用 ARM64 预编译版本
- 性能优化策略与 x86 平台有所差异

---

## 前期准备

### 1. 确认系统信息

```bash
# 查看 JetPack 版本
cat /etc/nv_tegra_release

# 查看系统架构（应显示 aarch64）
uname -m

# 查看 Ubuntu 版本
lsb_release -a

# 查看 CUDA 版本
nvcc --version
```

### 2. 更新系统

```bash
sudo apt update
sudo apt upgrade -y
```

### 3. 确保有足够的存储空间

```bash
# 查看磁盘空间（建议至少 10GB 可用）
df -h

# 如果使用 SD 卡，建议挂载外部 SSD
# 可以将项目和编译输出放在 SSD 上以提高性能
```

### 4. 设置交换空间（Swap）

Jetson 内存有限，编译大型项目时可能需要额外的交换空间：

```bash
# 查看当前交换空间
free -h

# 如果交换空间不足 4GB，建议增加
sudo systemctl disable nvzramconfig

# 创建 4GB 交换文件
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 永久启用
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 第一步：安装海康威视 MVS SDK（ARM64版本）

### 1.1 下载 SDK

前往海康威视官网下载 **ARM64 版本** 的 MVS SDK：

**官方下载地址**：https://www.hikrobotics.com/cn2/source/support/software/

1. 登录或注册账号
2. 进入"工业相机" → "客户端软件"
3. 选择 **Linux ARM64** 版本
4. 下载文件，例如：`MVS-2.1.2_aarch64_20231116.tar.gz`

### 1.2 解压并安装

```bash
# 假设下载到 ~/Downloads 目录
cd ~/Downloads

# 解压文件
tar -zxvf MVS-2.1.2_aarch64_20231116.tar.gz
cd MVS-2.1.2_aarch64

# 安装依赖
sudo apt install -y libusb-1.0-0-dev

# 运行安装脚本
sudo ./install.sh
```

### 1.3 验证安装

```bash
# 检查安装路径
ls /opt/MVS/

# 应该看到以下目录：
# bin/  include/  lib/  Samples/  doc/

# 检查库文件
ls /opt/MVS/lib/64bit/libMvCameraControl.so

# 配置动态链接库路径
echo 'export LD_LIBRARY_PATH=/opt/MVS/lib/64bit:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 1.4 测试相机连接

```bash
# 连接海康相机后执行
lsusb | grep -i hikrobot

# 预期输出类似：
# Bus 001 Device 005: ID 2bdf:xxxx Hikrobot

# 运行官方示例程序测试
cd /opt/MVS/Samples/aarch64/BasicDemo
mkdir build && cd build
cmake ..
make
./BasicDemo
```

**注意**：
- 如果相机无法识别，检查 USB 端口供电是否充足
- Jetson 的 USB 端口可能功率有限，建议使用带外部供电的 USB Hub

---

## 第二步：安装依赖项

### 2.1 安装基础开发工具

```bash
sudo apt install -y \
    git \
    g++ \
    cmake \
    build-essential \
    pkg-config \
    wget \
    curl \
    unzip
```

### 2.2 安装 OpenCV

JetPack 通常预装了 OpenCV，但建议确认版本：

```bash
# 检查 OpenCV 版本
pkg-config --modversion opencv4

# 如果未安装或版本过低，可以重新编译或安装预编译版本
sudo apt install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv
```

**如果需要从源码编译 OpenCV（可选）**：

```bash
# 参考 Jetson 官方文档或使用社区脚本
# https://github.com/mdegans/nano_build_opencv
```

### 2.3 安装其他依赖

```bash
sudo apt install -y \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    can-utils \
    openssh-server \
    screen
```

### 2.4 安装 Ceres Solver

Ceres 需要从源码编译（ARM64 预编译版本较少）：

```bash
# 安装 Ceres 依赖
sudo apt install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev

# 下载 Ceres 源码
cd ~
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar -zxvf ceres-solver-2.1.0.tar.gz
cd ceres-solver-2.1.0

# 编译安装
mkdir build && cd build
cmake .. \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

**编译时间**：在 Jetson Orin NX 上约需 10-20 分钟，Nano 上可能需要 1 小时以上。

### 2.5 安装 OpenVINO（可选，用于神经网络推理加速）

**注意**：OpenVINO 对 ARM64 支持有限，Jetson 平台推荐使用 **TensorRT** 替代。

#### 方案 A：使用 TensorRT（推荐）

JetPack 已预装 TensorRT，检查版本：

```bash
dpkg -l | grep TensorRT
```

项目如需使用 TensorRT，需要将模型转换为 TensorRT 格式（.engine 文件）。

#### 方案 B：尝试安装 OpenVINO ARM64 版本

```bash
# OpenVINO 官方支持 ARM64，但性能不如 TensorRT
# 下载页面：https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html
# 选择 ARM64 版本
```

**建议**：如果项目使用 OpenVINO 模型，考虑：
1. 转换为 TensorRT 格式（最佳性能）
2. 使用 ONNX Runtime ARM64 版本
3. 或直接使用 OpenVINO ARM64 版本（性能次优）

---

## 第三步：编译项目

### 3.1 克隆项目

```bash
cd ~
git clone <your-repository-url> nyush-rm-vision
cd nyush-rm-vision
```

### 3.2 修改 CMakeLists.txt（如果需要）

检查 `CMakeLists.txt` 是否正确检测 ARM64 架构：

```cmake
# 在 CMakeLists.txt 中添加架构检测
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "Building for ARM64 (Jetson)")
    # ARM64 特定配置
    set(MVS_LIB_DIR /opt/MVS/lib/64bit)
    set(MVS_INCLUDE_DIR /opt/MVS/include)
endif()
```

### 3.3 编译

```bash
# 创建构建目录
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-march=native" \
    -DBUILD_TESTING=OFF

# 编译（使用所有CPU核心）
make -C build/ -j$(nproc)
```

**编译时间**：
- Jetson Orin NX：约 5-15 分钟
- Jetson Xavier NX：约 10-20 分钟
- Jetson Nano：约 30-60 分钟（建议增加 swap）

### 3.4 解决编译错误（如果有）

常见问题：

#### 问题 1：找不到 MVS 库

```bash
# 确保环境变量正确
export MVS_DIR=/opt/MVS
export LD_LIBRARY_PATH=/opt/MVS/lib/64bit:$LD_LIBRARY_PATH
```

#### 问题 2：内存不足

```bash
# 减少并行编译任务数
make -C build/ -j2  # 使用2个核心编译
```

#### 问题 3：OpenVINO 相关错误

如果不使用 OpenVINO，可以在 CMakeLists.txt 中禁用相关功能。

---

## 第四步：相机测试

### 4.1 配置相机权限

```bash
# 创建 udev 规则
sudo nano /etc/udev/rules.d/99-hikrobot-camera.rules

# 添加以下内容
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666"

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 将用户加入 video 组
sudo usermod -a -G video $USER

# 重新登录使权限生效
```

### 4.2 运行相机测试

```bash
# 基本相机测试
./build/camera_test configs/calibration.yaml

# 如果成功，应该能看到实时图像窗口
```

### 4.3 检查相机参数

```bash
# 查看相机 VID:PID
lsusb | grep Hikrobot

# 示例输出：
# Bus 001 Device 005: ID 2bdf:0001 Hikrobot

# 在配置文件中设置
# vid_pid: "2bdf:0001"
```

---

## 第五步：性能优化

### 5.1 启用最大性能模式

Jetson 默认可能运行在节能模式，启用最大性能模式：

```bash
# 查看当前功耗模式
sudo nvpmodel -q

# 设置为最大性能模式
# Jetson Orin NX：MAXN (15W 或 25W)
sudo nvpmodel -m 0

# 启用最大 CPU 频率
sudo jetson_clocks

# 查看当前频率和温度
tegrastats
```

### 5.2 使用 CUDA 加速（如果支持）

如果项目中使用了 CUDA 加速模块：

```bash
# 确认 CUDA 可用
nvcc --version

# 在编译时启用 CUDA
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_CUDA=ON \
    -DCMAKE_CUDA_ARCHITECTURES=87  # Orin 架构为 87，Xavier 为 72
```

### 5.3 使用 TensorRT 加速推理

如果项目使用神经网络推理：

1. **转换模型为 TensorRT 格式**：

```bash
# 使用 trtexec 工具转换 ONNX 模型
/usr/src/tensorrt/bin/trtexec \
    --onnx=model.onnx \
    --saveEngine=model.engine \
    --fp16  # 启用 FP16 加速
```

2. **修改代码使用 TensorRT 引擎**（需要代码适配）

### 5.4 优化图像处理

```bash
# 降低图像分辨率（在配置文件中）
# 例如从 1440x1080 降至 1280x720

# 启用硬件加速的图像处理
# Jetson 的 VPI (Vision Programming Interface) 可以加速某些操作
```

### 5.5 监控性能

```bash
# 实时监控 CPU、GPU、内存、温度
tegrastats

# 或使用 jtop（推荐）
sudo pip3 install jetson-stats
jtop
```

---

## 常见问题排查

### Q1: 相机无法打开或识别失败

**症状**：程序提示 "Camera open failed" 或找不到设备

**排查步骤**：

```bash
# 1. 确认相机连接
lsusb | grep -i hikrobot

# 2. 检查 SDK 安装
ls /opt/MVS/lib/64bit/libMvCameraControl.so

# 3. 检查动态链接库
ldd ./build/camera_test | grep MVS

# 4. 检查权限
ls -l /dev/bus/usb/001/*  # 替换为实际 bus 号

# 5. 测试官方示例
cd /opt/MVS/Samples/aarch64/BasicDemo/build
./BasicDemo
```

**解决方案**：
- 使用带外部供电的 USB Hub
- 检查 USB 线缆质量
- 尝试不同的 USB 端口（优先使用 USB 3.0）
- 确认 udev 规则生效

### Q2: 编译时内存不足

**症状**：编译过程中系统卡死或报错 "virtual memory exhausted"

**解决方案**：

```bash
# 增加 swap 空间（见前期准备章节）
# 减少并行编译数
make -C build/ -j1  # 单线程编译

# 或者使用交叉编译（在 x86 机器上编译 ARM64 程序）
```

### Q3: 运行时帧率过低

**症状**：相机采集或处理帧率低于预期（<30 FPS）

**排查**：

```bash
# 1. 检查功耗模式
sudo nvpmodel -q
sudo jetson_clocks

# 2. 监控资源占用
jtop  # 查看 CPU、GPU 使用率

# 3. 检查是否使用了硬件加速
# 确认 CUDA、TensorRT 是否正确启用
```

**优化方案**：
- 降低图像分辨率
- 使用 TensorRT 而非 OpenVINO
- 启用多线程处理
- 优化神经网络模型（量化、剪枝）
- 减少日志输出

### Q4: 温度过高或降频

**症状**：运行一段时间后性能下降，`tegrastats` 显示温度过高

**解决方案**：

```bash
# 1. 添加散热器或风扇
# 主动散热可以显著降低温度

# 2. 限制功耗模式
# 如果温度持续过高，选择较低的功耗模式
sudo nvpmodel -m 1  # 选择次高性能模式

# 3. 监控温度
watch -n 1 cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

### Q5: CAN 或串口通信失败

**症状**：无法与下位机通信

**排查 CAN**：

```bash
# 1. 检查 CAN 设备
ip link show can0

# 2. 确认 CAN 启动
sudo ip link set can0 up type can bitrate 1000000

# 3. 测试 CAN 通信
candump can0
```

**排查串口**：

```bash
# 1. 检查串口设备
ls /dev/ttyTHS*  # Jetson 硬件串口
ls /dev/ttyACM*  # USB 虚拟串口

# 2. 测试串口通信
sudo minicom -D /dev/ttyTHS1

# 3. 设置权限
sudo chmod 666 /dev/ttyTHS1
```

### Q6: OpenVINO 不可用

**症状**：编译或运行时报 OpenVINO 相关错误

**解决方案**：

1. **禁用 OpenVINO**（推荐）：
   - 在 CMakeLists.txt 中关闭 OpenVINO 选项
   - 使用 TensorRT 替代

2. **安装 OpenVINO ARM64 版本**：
   - 从官网下载 ARM64 版本
   - 注意性能可能不如 TensorRT

3. **使用 ONNX Runtime**：
   ```bash
   sudo apt install -y libonnxruntime-dev
   ```

---

## Jetson 特定注意事项

### 1. 电源管理

- **供电要求**：确保使用官方推荐的电源适配器
- **外设功耗**：工业相机可能需要额外供电，避免直接从 Jetson USB 取电
- **功耗模式**：根据散热条件选择合适的 nvpmodel 模式

### 2. 存储选择

- **SD 卡**：读写速度慢，建议仅用于系统盘
- **NVMe SSD**：强烈推荐，显著提升编译和运行性能
- **USB SSD**：次优选择，注意 USB 带宽限制

### 3. 散热方案

- **被动散热**：Orin Nano 等小型模块可能需要额外散热器
- **主动散热**：推荐使用 PWM 风扇，Jetson 可自动调速
- **环境温度**：RoboMaster 比赛场地可能高温，注意散热设计

### 4. 网络配置

Jetson 用于机器人时通常需要配置无线网络：

```bash
# 设置静态 IP
sudo nmcli con mod Wired\ connection\ 1 ipv4.addresses 192.168.1.100/24
sudo nmcli con mod Wired\ connection\ 1 ipv4.method manual

# 或配置 WiFi
sudo nmcli device wifi connect "SSID" password "PASSWORD"
```

### 5. 远程访问

```bash
# 安装 NoMachine（推荐）
# 下载 ARM64 版本：https://www.nomachine.com/download/linux&id=30

# 或使用 SSH + X11 转发
ssh -X user@jetson-ip

# 使用 VNC（备选）
sudo apt install vino
```

### 6. 实时性优化

如果需要更好的实时性能：

```bash
# 禁用桌面环境（减少资源占用）
sudo systemctl set-default multi-user.target

# 设置进程优先级
sudo nice -n -20 ./build/your_program

# 绑定 CPU 核心
taskset -c 0,1 ./build/your_program
```

### 7. 系统监控脚本

创建监控脚本 `monitor.sh`：

```bash
#!/bin/bash
echo "===== Jetson Status ====="
echo "Power Mode: $(sudo nvpmodel -q | grep 'Mode')"
echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}')"
echo "Memory: $(free -h | grep Mem | awk '{print $3 "/" $2}')"
echo "Temperature: $(cat /sys/devices/virtual/thermal/thermal_zone0/temp | awk '{print $1/1000 "°C"}')"
echo "GPU Usage: $(tegrastats --interval 1000 | head -1)"
```

---

## 部署检查清单

在正式部署前，确认以下事项：

- [ ] Jetson 系统版本和 JetPack 版本正确
- [ ] 海康威视 SDK ARM64 版本已安装
- [ ] 所有依赖项已正确安装
- [ ] 项目编译成功，无警告和错误
- [ ] 相机测试通过，能够正常采集图像
- [ ] 下位机通信（CAN 或串口）测试通过
- [ ] 标定程序运行正常
- [ ] 性能模式设置为 MAXN
- [ ] 散热方案充足，温度在安全范围内
- [ ] 自启动脚本配置正确
- [ ] 远程访问（SSH/NoMachine）配置完成
- [ ] 日志记录和监控工具就绪

---

## 参考资源

### 官方文档

- **NVIDIA Jetson 官方文档**：https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit
- **JetPack SDK**：https://developer.nvidia.com/embedded/jetpack
- **海康威视 MVS SDK**：https://www.hikrobotics.com/cn2/source/support/software/

### 社区资源

- **Jetson Community Forums**：https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
- **JetsonHacks**：https://jetsonhacks.com/
- **Jetson-Stats (jtop)**：https://github.com/rbonghi/jetson_stats

### 相关项目

- 项目主 README：`readme.md`
- x86 部署指南：`DEPLOYMENT_GUIDE.md`
- 下位机通信协议：`下位机通信协议文档.md`

---

## 联系与支持

如果在 Jetson 部署过程中遇到问题：

1. 查阅本文档的"常见问题排查"章节
2. 检查 Jetson 官方论坛和 JetsonHacks 教程
3. 提交 Issue 到项目仓库，注明：
   - Jetson 型号和 JetPack 版本
   - 详细错误信息和日志
   - 已尝试的解决方案

---

**最后更新**：2026-02-08

**文档版本**：v1.0

**适用平台**：Jetson Orin/Xavier/Nano + JetPack 5.x/6.x + 海康威视工业相机
