# 串口数据调试指南

## 快速开始

### 方式1：直接运行调试工具

```bash
cd /home/hero/DUST_Hero
sudo ./serial_debug
```

这会显示下位机发送的所有数据包的十六进制内容和分析结果。

## 数据格式说明

### 预期的帧格式

```
[Byte 0]  Frame Header (SOF)    - 应该是 0xA5
[Byte 1]  CRC8                   - 帧头校验
[Bytes 2-N]  InputData           - 实际数据（大小取决于机器人型号）
[Byte N+1]   CRC16 (Low byte)    - 帧尾校验
[Byte N+2]   CRC16 (High byte)   - 帧尾校验
```

### 英雄(TJURM_HERO)的InputData结构

```
Offset  Size   Field
------  ----   -----
0       4      curr_yaw (float)
4       4      curr_pitch (float)
8       1      state (uint8_t)
9       1      autoaim (uint8_t)
10      1      enemy_color (uint8_t)
11      4      curr_speed (float)
-----
总共: 15 字节
```

**完整帧大小 = 1(sof) + 1(crc8) + 15(data) + 2(crc16) = 19 字节**

## 调试清单

运行调试工具后，检查以下内容：

### ✓ 帧头检查
- [ ] 第一个字节是否为 `0xA5` ?
- 如果不是，说明下位机发送的帧头不对

### ✓ 数据包大小
- [ ] 每个数据包是否都是 19 字节（英雄）？
- [ ] 是否有数据截断或数据重复？

### ✓ CRC8 校验
- [ ] CRC8 值是否合理（0x00-0xFF）？
- [ ] 不同包的CRC8是否都不同？

### ✓ 数据内容
- [ ] yaw/pitch 值是否在合理范围内（±180°）？
- [ ] state 值是否在合理范围内（0-3）？
- [ ] enemy_color 是否为 0 或 1？
- [ ] curr_speed 是否接近射速（通常15-20）？

### ✓ CRC16 校验
- [ ] CRC16 值是否与数据包内容相关？

## 常见问题诊断

### 问题1: 看不到任何数据
**可能原因：**
- 下位机没有发送数据
- 波特率不匹配（当前设置为115200）
- 串口连接不对

**解决方案：**
```bash
# 检查串口是否存在
ls -l /dev/ttyACM*

# 查看串口属性
stty -a < /dev/ttyACM0
```

### 问题2: 帧头不是0xA5
**可能原因：**
- 下位机使用不同的帧头字节
- 波特率错误导致数据错位

**解决方案：**
- 记下实际接收到的帧头字节
- 修改代码中的 `0xA5` 为正确的值

### 问题3: 数据包大小不对
**可能原因：**
- 机器人型号与编译时的宏定义不符
- 下位机发送了额外的数据或帧结尾标记

**解决方案：**
- 核实 CMakeLists.txt 中编译的机器人型号
- 检查是否有帧尾的 `\n` 字符

### 问题4: CRC16错误
**可能原因：**
- CRC16算法不匹配
- 数据在传输过程中损坏

**解决方案：**
- 检查 src/threads/control/crc.cpp 中的CRC16算法
- 与下位机的CRC16实现进行对比

## 比较建议

如果下位机和上位机的数据包格式不匹配，需要：

1. **确认帧头字节** - 修改 `include/threads/control.h` 中的 `#define SOF 0xA5`
2. **确认数据字段顺序和大小** - 修改 `include/threads/control/structure.h` 中的结构体
3. **确认CRC算法** - 修改 `src/threads/control/crc.cpp` 中的验证函数
4. **确认波特率** - 修改 `serial_debug.cpp` 中的 `B115200` 或 `setup_serial_port` 函数

