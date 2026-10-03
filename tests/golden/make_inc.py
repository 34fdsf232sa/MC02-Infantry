#!/usr/bin/env python3
"""golden.txt（gen_f4_vectors 的输出）-> C++ 初始化列表，供 test_mit_motor.cpp #include。

行格式：
  ENC pmax vmax tmax  pos vel kp kd torque  <8 字节帧 hex>
  DEC pmax vmax tmax  <8 字节帧 hex>  位置 速度 力矩 MOS温度 转子温度 状态
"""
import sys


def lit(s):
    """数值字符串 -> float 字面量（%.9g 已能无损往返 float，加 f 后缀避免当成 double）"""
    return (s if any(c in s for c in ".e") else s + ".0") + "f"


def frame(h):
    return ", ".join("0x" + h[i:i + 2] for i in range(0, 16, 2))


enc, dec = [], []
for line in open(sys.argv[1]):
    f = line.split()
    if not f:
        continue
    if f[0] == "ENC":
        pm, vm, tm, pos, vel, kp, kd, tq = (lit(x) for x in f[1:9])
        enc.append(f"    {{{pm}, {vm}, {tm}, {pos}, {vel}, {kp}, {kd}, {tq}, {{{frame(f[9])}}}}},")
    elif f[0] == "DEC":
        pm, vm, tm = (lit(x) for x in f[1:4])
        pos, vel, tq = (lit(x) for x in f[5:8])
        mos, rotor, status = int(float(f[8])), int(float(f[9])), int(f[10])
        dec.append(f"    {{{pm}, {vm}, {tm}, {{{frame(f[4])}}}, {pos}, {vel}, {tq}, {mos}, {rotor}, {status}}},")

print("// 自动生成，勿手改。来源：原版 F4 固件 mi_motor.c（见 gen_f4_vectors.c 顶部说明）")
print("#ifdef MIT_GOLDEN_ENC")
print("\n".join(enc))
print("#endif")
print("#ifdef MIT_GOLDEN_DEC")
print("\n".join(dec))
print("#endif")
