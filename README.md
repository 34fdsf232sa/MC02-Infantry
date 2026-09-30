# MC02-Infantry

轮腿平衡步兵固件，上下板都是达妙 DM-MC02（STM32H723VGT6）。

| 板 | 负责 |
|---|---|
| 底盘 | 4× 铁蛋关节（MIT 标准帧）、2× M3508 + 定制减速（C620）、BMI088、超电、ToF、DJI 裁判系统（经滑环） |
| 云台 | yaw/pitch DM4310、摩擦轮 2× M3508、拨弹（M2006/M3508 待定）、Epson G354 + 硬同步载板、USB CDC ↔ NUC |

## 目录

```
bsp/mc02/        CubeMX 工程（由达妙官方 CtrBoard-H7_ALL.ioc 转 CMake 生成，只改 USER CODE 段）  [待建]
control/         纯算法，无硬件依赖：五连杆 VMC ✅、观测器、LQR、PID、AHRS
drivers/         电机/传感器协议编解码，与总线解耦                                          [待建]
framework/       离线检测、双板链路、上位机链路、参数存储                                     [待建]
apps/chassis/    底盘板任务编排                                                               [待建]
apps/gimbal/     云台板任务编排                                                               [待建]
tools/matlab/    轮腿建模与 K(L0) 拟合                                                         [待建]
tests/           主机单测（GoogleTest）
cmake/           交叉编译工具链
```

## 设计原则

1. 算法和协议编解码不碰 HAL，同一份源码既进固件也在主机上单测。
2. 模块之间用 topic 发布订阅，不共享全局结构体。
3. 每个外部输入（电机反馈、IMU、双板、遥控、裁判）都有超时检测；超时的模块进入安全态，底盘安全态为关节和轮子零力矩。
4. 控制任务按固定周期运行（`vTaskDelayUntil`），发送路径里不允许阻塞延时。
5. 只拷贝 MIT / BSD / Apache / ISC 许可的代码；没有许可证的项目只参考思路。

## 借鉴来源

| 来源 | 借鉴内容 | 许可证 |
|---|---|---|
| xrobot-org/libxr | 外设与 OS 抽象、Topic、H7 缓存处理、USB 协议栈 | Apache-2.0 |
| QDU-Robomaster 模块（Wheelleg 等） | 模块划分、轮腿控制结构 | 无，仅参考思路 |
| HNUYueLuRM/basic_framework | daemon 离线检测、can_comm 双板分包、BMI088 温控 + QuaternionEKF | MIT |
| RoboMaster-DLMU-CONE/one-framework、rpl | 类型化收发包、加热 PID、单测与 CI | BSD-3 / ISC |
| WilliamGwok/RP_Balance | 轮腿建模、K(Ll, Lr) 拟合脚本 | MIT |
| Epson imu_linux_example | G354 burst 读取、DRDY、外部同步配置 | Epson 许可 |

## 主机单测

```bash
cmake -S . -B build/host && cmake --build build/host -j && ctest --test-dir build/host
```

## 交叉编译

```bash
cmake -S . -B build/m7 -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-cortex-m7.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/m7
```

## 原 F4 代码中没有带过来的问题

- `Get_Leg_Force` 用 J⁻¹ 求摆杆力，正确做法是 (Jᵀ)⁻¹。
- `Calculate_VMC` 忽略了传入的 F0/Tp 参数，实际读的是全局变量。
- 双板在线检测把状态写到了 `RC` 上，而且检测函数从未被调用。
- 控制发送路径中有 8 次 `osDelay(1)`，实际控制周期约 10 ms。
