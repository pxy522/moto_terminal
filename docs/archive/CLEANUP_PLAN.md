# 项目架构重组方案

## 🎯 目标

1. 删除无用文件
2. 重组文档结构
3. 简化项目架构
4. 保留核心功能

---

## 📊 当前问题

### 无用文件
- ❌ `TimerManager.*` - 未被使用（已改用 FreeRTOS 直接管理）
- ❌ `JY901.h/cpp` - 已被 `REG_JY901` 替代
- ❌ `FreeRTOS_Config_Guide.h` - 只是文档，不应在 include 目录
- ❌ `TimerManager_Config.h` - TimerManager 未使用

### 文档混乱
- 太多文档文件（5个 MD）
- 根目录有 3 个 MD 文件
- 文档之间有重复内容

### 旧代码残留
- main.cpp 中有大量调试模式代码
- 保留了 3 套算法（只需要模式 3）

---

## ✅ 重组方案

### 阶段 1: 删除无用文件

#### 删除列表

**源文件（未使用）:**
```
include/TimerManager.h
include/TimerManager_Config.h
include/JY901.h
include/FreeRTOS_Config_Guide.h
src/TimerManager.cpp
src/JY901.cpp
```

**保留但整理的文档:**
```
docs/
├── README.md                    # 新：项目总览（替代根目录 README）
├── Quick_Start.md              # 快速开始（合并 QUICKSTART）
├── IMU_Algorithm_Guide.md      # IMU 算法指南（合并 3 个 IMU 文档）
└── archive/                     # 归档旧文档
    ├── MIGRATION_GUIDE.md
    ├── IMU_Fusion_Guide.md     # 旧 Madgwick 文档
    └── JY901_Problem_Analysis.md
```

### 阶段 2: 简化代码

#### main.cpp 简化
- 删除调试模式 0, 1, 2（保留模式 3）
- 删除旧的 `compensateGravity()` 函数
- 删除诊断函数（可选保留一个简化版）
- 删除测试函数

#### DataProcessor.cpp 简化
- 删除旧的 `DP_jy901()` 函数
- 保留 `DP_jy901_Motorcycle()`（如果还在用）

### 阶段 3: 新项目结构

```
moto_terminal_freertos/
├── .vscode/
├── .pio/
├── docs/
│   ├── README.md                   # 📖 项目文档主页
│   ├── Quick_Start.md              # 🚀 快速开始
│   ├── IMU_Algorithm.md            # 🧭 IMU 算法说明
│   ├── Hardware_Setup.md           # 🔌 硬件连接（新建）
│   └── archive/                    # 📦 历史文档归档
│       ├── MIGRATION_GUIDE.md
│       ├── old_madgwick_guide.md
│       └── problem_analysis.md
├── include/                        # 头文件
│   ├── 485SN.h
│   ├── BLE.h
│   ├── DataProcessor.h
│   ├── DFRobot_AS7341.h
│   ├── IMU_Fusion.h               # ✅ 核心：IMU 融合
│   ├── REG_JY901.h                # ✅ 核心：JY901 驱动
│   ├── SD_card.h
│   ├── TCA9548A.h
│   └── wifi_wrapper.h
├── src/                            # 源文件
│   ├── 485SN.cpp
│   ├── BLE.cpp
│   ├── DataProcessor.cpp
│   ├── DFRobot_AS7341.cpp
│   ├── IMU_Fusion.cpp             # ✅ 核心：IMU 融合实现
│   ├── main.cpp                    # ✅ 核心：主程序（简化后）
│   ├── REG_JY901.cpp              # ✅ 核心：JY901 驱动
│   ├── SD_card.cpp
│   ├── TCA9548A.cpp
│   └── wifi_wrapper.cpp
├── platformio.ini                  # 构建配置
└── README.md                       # 项目简介（新）
```

---

## 🗑️ 要删除的文件清单

### 头文件（4个）
- [ ] `include/TimerManager.h`
- [ ] `include/TimerManager_Config.h`
- [ ] `include/JY901.h`
- [ ] `include/FreeRTOS_Config_Guide.h`

### 源文件（2个）
- [ ] `src/TimerManager.cpp`
- [ ] `src/JY901.cpp`

### 文档（移到 archive）
- [ ] `MIGRATION_GUIDE.md` → `docs/archive/`
- [ ] `QUICKSTART.md` → 合并到新文档
- [ ] 根目录 `README.md` → 简化重写

### 文档（合并）
- [ ] `docs/IMU_Fusion_Guide.md` → 合并到 `docs/IMU_Algorithm.md`
- [ ] `docs/JY901_Problem_Analysis.md` → 合并到 `docs/IMU_Algorithm.md`
- [ ] `docs/Correct_IMU_Algorithm_Guide.md` → 合并到 `docs/IMU_Algorithm.md`
- [ ] `docs/IMU_Frequency_Analysis.md` → 作为附录
- [ ] `docs/100Hz_Upgrade_Notes.md` → 作为 changelog

---

## 📝 新文档结构

### 1. README.md（根目录 - 新）
- 项目简介
- 核心特性
- 快速开始链接
- 系统架构图

### 2. docs/Quick_Start.md
- 硬件连接
- 软件安装
- 编译上传
- 基本配置

### 3. docs/IMU_Algorithm.md
- 为什么用模式 3
- JY901 工作原理
- 四元数重力补偿
- 参数调优
- 100Hz 配置说明

### 4. docs/Hardware_Setup.md（新）
- ESP32 引脚定义
- I2C 设备地址
- 传感器连接图
- 故障排除

---

## 🔧 代码简化方案

### main.cpp 简化
```cpp
// 删除：
- debug_mode 0, 1, 2, 4（只保留模式 3）
- compensateGravity() 旧函数
- processIMU_NewAlgorithm() Madgwick 函数
- IMU_DiagnosePrint() 旧诊断
- IMU_CompareAlgorithms() 对比测试
- Test_GravityCompensation_AllCombinations()
- Check_CoordinateSystem()
- Test_TiltCompensation()
- Test_MotionDirection()

// 保留：
- processIMU_Correct() ✅ 核心算法
- IMU_DiagnosePrint_Correct() ✅ 简化版诊断（可选）
- 所有任务函数
```

**预计删除代码:** ~600 行

### DataProcessor.cpp 简化
```cpp
// 删除：
- DP_jy901() 旧函数
- DP_jy901_SimpleTest() 测试函数
- removeGravity() 旧函数（已移到 IMU_Fusion）

// 保留：
- DP_jy901_Motorcycle() 如果还在用
- Motorcycle_GetSmoothSpeed()
- 手套数据处理函数
```

---

## ⚡ 执行步骤

### 步骤 1: 备份（安全起见）
```bash
cd /c/Users/SRichael/Desktop/projects/moto_terminal_freertos
cp -r . ../moto_terminal_freertos_backup
```

### 步骤 2: 删除无用文件
```bash
# 删除 TimerManager
rm include/TimerManager.h include/TimerManager_Config.h
rm src/TimerManager.cpp

# 删除旧 JY901
rm include/JY901.h src/JY901.cpp

# 删除配置指南
rm include/FreeRTOS_Config_Guide.h
```

### 步骤 3: 重组文档
```bash
# 创建归档目录
mkdir -p docs/archive

# 移动旧文档
mv MIGRATION_GUIDE.md docs/archive/
mv QUICKSTART.md docs/archive/

# 重命名文档
mv docs/JY901_Problem_Analysis.md docs/archive/problem_analysis.md
mv docs/IMU_Fusion_Guide.md docs/archive/old_madgwick_guide.md
```

### 步骤 4: 创建新文档
- 新的 README.md
- docs/Quick_Start.md
- docs/IMU_Algorithm.md
- docs/Hardware_Setup.md

### 步骤 5: 简化代码
- 编辑 main.cpp（删除旧模式）
- 编辑 DataProcessor.cpp（删除旧函数）

### 步骤 6: 测试
```bash
pio run -t clean
pio run
```

---

## 📊 预期结果

### 文件数量
| 类型 | 当前 | 清理后 | 减少 |
|------|------|--------|------|
| 头文件 | 13 | 9 | -4 |
| 源文件 | 12 | 10 | -2 |
| 文档（docs） | 5 | 4 + archive | 整理 |
| 根目录文档 | 3 | 1 | -2 |
| **总计** | **33** | **24** | **-9** |

### 代码行数
| 文件 | 当前 | 简化后 | 减少 |
|------|------|--------|------|
| main.cpp | ~980 | ~400 | -580 |
| DataProcessor.cpp | ~500 | ~400 | -100 |

### 磁盘占用
- 删除冗余代码和文档
- 减少约 **20%** 文件数量
- 保持所有核心功能

---

## ✅ 优势

1. **结构清晰** - 只保留使用中的文件
2. **文档整理** - 归档历史，突出核心
3. **代码简洁** - 删除实验性代码
4. **易于维护** - 减少混乱和重复
5. **新人友好** - 清晰的项目结构

---

## ⚠️ 注意事项

1. **备份** - 清理前务必备份
2. **测试** - 清理后重新编译测试
3. **Git** - 如果用 Git，先提交当前状态
4. **分阶段** - 可以先试删除一部分，测试无误后继续

---

**准备好了吗？我可以帮你执行清理！**
