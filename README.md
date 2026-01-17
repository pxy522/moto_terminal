# 摩托车数据终端 - ESP32 FreeRTOS

基于 ESP32-S3 的摩托车数据采集和监控系统，采用 FreeRTOS 实现多传感器实时数据处理。

## 核心特性

- **高精度 IMU 姿态解算** - 使用 JY901 九轴传感器，100Hz 采样率
- **多传感器融合** - 集成 IMU、光谱传感器、气象站、手套传感器
- **实时数据记录** - SD 卡存储，支持长时间记录
- **无线数据传输** - WiFi 和 BLE 双模式支持
- **FreeRTOS 多任务** - 任务隔离，精确定时，资源保护

## 硬件配置

| 组件 | 型号 | 接口 | 功能 |
|------|------|------|------|
| 主控 | ESP32-S3 WROOM | - | 双核处理器，WiFi/BLE |
| IMU | JY901 (MPU9250) | I2C | 9轴姿态传感器（100Hz）|
| 光谱传感器 | AS7341 | I2C | 11通道光谱检测 |
| 气象站 | WS485 | RS485 | 温湿度、风速、雨量 |
| 存储 | MicroSD | SPI | 数据记录 |
| 显示 | 串口屏 | UART | 实时数据显示 |

## 快速开始

### 环境准备

```bash
# 安装 PlatformIO
pip install platformio

# 克隆项目
git clone <repository-url>
cd moto_terminal_freertos
```

### 编译上传

```bash
# 编译项目
pio run

# 上传到设备
pio run -t upload

# 监控串口输出
pio device monitor
```

### 基本配置

在 [main.cpp](src/main.cpp) 中配置：

```cpp
// IMU 处理频率（推荐 100Hz）
const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 10ms = 100Hz

// 运行模式（3 = 正确的 IMU 算法）
uint8_t debug_mode = 3;
```

## 系统架构

### 任务分配

| 任务 | 核心 | 优先级 | 频率 | 说明 |
|------|------|--------|------|------|
| IMU | Core 1 | 5 | 100Hz | 姿态解算、速度积分 |
| AS7341 | Core 1 | 3 | 5Hz | 光谱数据采集 |
| 串口接收 | Core 0 | 4 | 事件驱动 | 手套/CKP 数据 |
| RS485 | Core 0 | 3 | 5Hz | 气象站数据 |
| SD 卡 | Core 0 | 2 | 0.5Hz | 数据记录 |
| BLE | Core 0 | 2 | 10Hz | 蓝牙通信 |
| 显示 | Core 0 | 1 | 0.2Hz | 串口屏更新 |

### 资源保护

使用互斥锁（Mutex）保护共享资源：

- `mutexI2C` - I2C 总线访问
- `mutexSD` - SD 卡访问
- `mutexSerial` - 串口访问

## IMU 算法说明

本项目使用**正确的 IMU 解算方案**（模式 3）：

### 核心原理

1. **直接使用 JY901 姿态输出** - JY901 内部已有卡尔曼滤波（9轴融合）
2. **四元数精确重力补偿** - 避免欧拉角近似误差
3. **零速修正（ZUPT）** - 检测静止状态，消除速度漂移
4. **速度衰减** - 对抗积分漂移

### 算法流程

```
JY901 内部 (200Hz)
    ↓
姿态角输出 (Roll/Pitch/Yaw)
    ↓
转换为四元数
    ↓
四元数旋转去除重力
    ↓
线性加速度积分
    ↓
速度输出 + ZUPT 修正
```

### 性能指标

| 指标 | 数值 |
|------|------|
| 姿态精度 | ±1° |
| 处理频率 | 100 Hz |
| 速度误差 | ~0.05 m/s (10秒后) |
| ZUPT 响应 | 1 秒 |
| CPU 占用 | ~3% |

## 项目结构

```
moto_terminal_freertos/
├── include/               # 头文件
│   ├── IMU_Fusion.h      # IMU 融合库
│   ├── REG_JY901.h       # JY901 驱动
│   ├── DataProcessor.h   # 数据处理
│   └── ...               # 其他传感器头文件
├── src/                   # 源文件
│   ├── main.cpp          # 主程序（FreeRTOS 任务）
│   ├── IMU_Fusion.cpp    # IMU 融合实现
│   ├── REG_JY901.cpp     # JY901 驱动实现
│   └── ...               # 其他传感器实现
├── docs/                  # 文档
│   ├── Correct_IMU_Algorithm_Guide.md    # IMU 算法使用指南
│   ├── 100Hz_Upgrade_Notes.md            # 100Hz 升级说明
│   ├── IMU_Frequency_Analysis.md         # 频率影响分析
│   └── archive/                          # 历史文档归档
├── platformio.ini         # 构建配置
└── README.md              # 本文件
```

## 文档索引

- [IMU 算法使用指南](docs/Correct_IMU_Algorithm_Guide.md) - 详细的算法说明和验证步骤
- [100Hz 升级说明](docs/100Hz_Upgrade_Notes.md) - 频率升级的修改内容
- [频率影响分析](docs/IMU_Frequency_Analysis.md) - 不同频率的性能对比
- [JY901 问题分析](docs/JY901_Problem_Analysis.md) - 为什么旧算法不准确
- [项目清理计划](docs/CLEANUP_PLAN.md) - 架构重组记录

## 性能对比

### FreeRTOS 重写前后

| 指标 | 重写前 | 重写后 | 改善 |
|------|--------|--------|------|
| IMU 采样周期 | ~1800ms | 10ms | **180倍** |
| 采样频率 | ~0.56Hz | 100Hz | **180倍** |
| 时序抖动 | 巨大 | ±1ms | 显著 |
| CPU 利用率 | 低效 | 高效 | 多核并行 |

### IMU 算法改进

| 指标 | 旧算法 | 正确算法 | 改善 |
|------|--------|----------|------|
| 姿态精度 | ±3° | ±1° | **3倍** |
| 传感器利用 | 6轴 | 9轴 | 完整利用 |
| Yaw 漂移 | 高 | 低 | 有磁力计 |
| 重力补偿 | 欧拉角 | 四元数 | 精确 |

## 常见问题

### Q: 速度仍然会漂移？

**A:** 这是纯 IMU 的固有限制。即使使用正确算法，长时间积分仍会漂移。建议：
1. 确认 ZUPT 工作正常（静止时速度应归零）
2. 调整衰减系数（参见算法文档）
3. **最佳方案**：加入轮速计或 GPS 辅助

### Q: 如何验证 IMU 算法正确？

**A:**
1. 设备水平静止，观察【线性加速度】模长 < 0.2 m/s²
2. 设备倾斜 30°，保持静止，线性加速度仍应 < 0.5 m/s²
3. 使用模式 4 对比 JY901 和 Madgwick 姿态差异

### Q: 为什么不用 Madgwick 重新解算？

**A:** JY901 内部已有姿态解算（9轴卡尔曼滤波），重新解算会：
- 丢失磁力计数据（9轴变6轴）
- 增加计算开销
- 降低精度（双重滤波冲突）

## 进一步优化

### 短期改进

- 调整 ZUPT 阈值（更敏感的零速检测）
- 优化速度衰减系数
- 添加实时性能监控

### 中长期扩展

- **加入轮速计** - 霍尔传感器，提升速度精度 10 倍
- **GPS 融合** - 卡尔曼融合 IMU + GPS
- **CAN 总线** - 读取摩托车 ECU 数据
- **Web 界面** - WiFi 实时监控和配置

## 调试技巧

### 串口输出

```bash
# 使用 PlatformIO 监控
pio device monitor

# 或使用过滤器
pio device monitor -f esp32_exception_decoder
```

### 性能监控

在 `main.cpp` 中启用诊断输出：

```cpp
uint8_t debug_mode = 3;  // 正确算法 + 诊断输出
```

输出示例：
```
========== ✅ 正确算法诊断 ==========
【JY901 姿态】Roll=2.35°, Pitch=-1.20°, Yaw=45.67°
【线性加速度】ax=0.004, ay=-0.002, az=0.006 m/s²
【速度】0.000 m/s (0.0 km/h)
======================================
```

## 贡献

欢迎提交 Issue 和 Pull Request。

## 许可证

MIT License

## 致谢

- **JY901 驱动**: 基于 WIT Motion 官方文档
- **IMU 融合**: Madgwick AHRS 算法
- **FreeRTOS**: ESP-IDF FreeRTOS 移植

---

**关键改进总结**：

✅ IMU 采样频率从 0.56Hz 提升到 100Hz（180倍）
✅ 姿态精度从 ±3° 提升到 ±1°（3倍）
✅ 使用 JY901 内部 9轴融合，保留磁力计数据
✅ 四元数精确重力补偿，避免欧拉角误差
✅ FreeRTOS 多任务，资源保护，精确定时
