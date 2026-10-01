#!/usr/bin/env python3
"""把达妙官方 CtrBoard-H7_ALL.ioc 调整为 LibXR 可用的配置。

只做 LibXR 要求的改动，时钟、引脚和其余外设参数保持官方原样：
  1. 所有 UART / SPI 开 DMA（D1/D2 域走 DMA1/DMA2，D3 域的 SPI6 走 BDMA）
  2. 开 SPI 全局中断（H7 HAL 的 SPI DMA 需要 EOT 中断收尾）
  3. HAL timebase (TIM23) 中断优先级改为 0：LibXR 微秒时间基要求更新中断不被读取方抢占
  4. 移除 CubeMX 自带 USB_DEVICE 中间件，USB_OTG_HS 交给 LibXR 自带的 USB 协议栈
  5. 只保留 defaultTask（栈 16 KB），增大 FreeRTOS 堆：LibXR 的 operator new 走 pvPortMalloc

用法：ioc_libxr_patch.py <in.ioc> <out.ioc>
幂等：对已打过补丁的文件再跑一次，输出不变。
"""

import re
import sys

# 请求顺序决定 Dma.RequestN 编号；DMA 流分配与 QDU-Robomaster/bsp-dev-mc02 一致
DMA_REQUESTS = [
    # (request, stream, direction)
    ("ADC1", None, None),  # 官方已有，保持原配置
    ("SPI1_TX", "DMA2_Stream6", "DMA_MEMORY_TO_PERIPH"),
    ("SPI2_RX", "DMA1_Stream2", "DMA_PERIPH_TO_MEMORY"),
    ("SPI2_TX", "DMA1_Stream3", "DMA_MEMORY_TO_PERIPH"),
    ("UART7_RX", "DMA1_Stream4", "DMA_PERIPH_TO_MEMORY"),
    ("UART7_TX", "DMA1_Stream5", "DMA_MEMORY_TO_PERIPH"),
    ("USART1_RX", "DMA1_Stream6", "DMA_PERIPH_TO_MEMORY"),
    ("USART1_TX", "DMA1_Stream7", "DMA_MEMORY_TO_PERIPH"),
    ("USART2_RX", "DMA2_Stream0", "DMA_PERIPH_TO_MEMORY"),
    ("USART2_TX", "DMA2_Stream1", "DMA_MEMORY_TO_PERIPH"),
    ("USART3_RX", "DMA2_Stream2", "DMA_PERIPH_TO_MEMORY"),
    ("USART3_TX", "DMA2_Stream3", "DMA_MEMORY_TO_PERIPH"),
    ("USART10_RX", "DMA2_Stream4", "DMA_PERIPH_TO_MEMORY"),
    ("USART10_TX", "DMA2_Stream5", "DMA_MEMORY_TO_PERIPH"),
]
BDMA_REQUESTS = [("SPI6_TX", "BDMA_Channel0", "DMA_MEMORY_TO_PERIPH")]

DMA_PARAMS = ("Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,"
              "Mode,Priority,FIFOMode,SignalID,Polarity,RequestNumber,SyncSignalID,"
              "SyncPolarity,SyncEnable,EventEnable,SyncRequestNumber")
BDMA_PARAMS = DMA_PARAMS.replace("FIFOMode,", "")

FREERTOS_HEAP = "0x14000"  # 80 KB，ucHeap 位于 DTCM（128 KB）

# 中断使能，抢占优先级 5（= configMAX_SYSCALL_INTERRUPT_PRIORITY，可调 FreeRTOS API）
IRQ_P5 = "true\\:5\\:0\\:false\\:false\\:true\\:true\\:false\\:true\\:true"


def dma_block(prefix, req, idx, stream, direction, with_fifo):
    lines = {
        "Direction": direction,
        "EventEnable": "DISABLE",
        "Instance": stream,
        "MemDataAlignment": "DMA_MDATAALIGN_BYTE",
        "MemInc": "DMA_MINC_ENABLE",
        "Mode": "DMA_NORMAL",
        "PeriphDataAlignment": "DMA_PDATAALIGN_BYTE",
        "PeriphInc": "DMA_PINC_DISABLE",
        "Polarity": "HAL_DMAMUX_REQ_GEN_RISING",
        "Priority": "DMA_PRIORITY_LOW",
        "RequestNumber": "1",
        "RequestParameters": DMA_PARAMS if with_fifo else BDMA_PARAMS,
        "SignalID": "NONE",
        "SyncEnable": "DISABLE",
        "SyncPolarity": "HAL_DMAMUX_SYNC_NO_EVENT",
        "SyncRequestNumber": "1",
        "SyncSignalID": "NONE",
    }
    if with_fifo:
        lines["FIFOMode"] = "DMA_FIFOMODE_DISABLE"
    return {f"{prefix}.{req}.{idx}.{k}": v for k, v in lines.items()}


def main(src, dst):
    kv = {}
    order = []
    with open(src, encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip("\n")
            if "=" not in line or line.startswith("#"):
                order.append((None, line))
                continue
            k, v = line.split("=", 1)
            kv[k] = v
            order.append((k, None))

    # ---- 1. DMA / BDMA ----
    # 清掉除 ADC1 外的旧 DMA 条目，按固定表重建，保证幂等
    for k in [k for k in kv if re.match(r"^(Dma|Bdma)\.", k)]:
        if not k.startswith("Dma.ADC1."):
            del kv[k]
    for idx, (req, stream, direction) in enumerate(DMA_REQUESTS):
        kv[f"Dma.Request{idx}"] = req
        if req == "ADC1":
            # ADC1 原条目索引就是 0，确认一下
            assert any(k.startswith("Dma.ADC1.0.") for k in kv), "ADC1 DMA 条目缺失"
            continue
        kv.update(dma_block("Dma", req, idx, stream, direction, with_fifo=True))
        kv[f"NVIC.{stream}_IRQn"] = IRQ_P5
    kv["Dma.RequestsNb"] = str(len(DMA_REQUESTS))
    for idx, (req, stream, direction) in enumerate(BDMA_REQUESTS):
        kv[f"Bdma.Request{idx}"] = req
        kv.update(dma_block("Bdma", req, idx, stream, direction, with_fifo=False))
        kv[f"NVIC.{stream}_IRQn"] = IRQ_P5
    kv["Bdma.RequestsNb"] = str(len(BDMA_REQUESTS))
    kv["NVIC.ForceEnableDMAVector"] = "true"

    # ---- 2. SPI 全局中断 ----
    for spi in ("SPI1", "SPI2", "SPI6"):
        kv[f"NVIC.{spi}_IRQn"] = IRQ_P5

    # ---- 3. timebase 优先级 0 ----
    tb = kv["NVIC.TimeBase"]
    assert tb == "TIM23_IRQn", f"意外的 timebase: {tb}"
    kv["NVIC.TIM23_IRQn"] = re.sub(r"^true\\:\d+\\:", "true\\:0\\:", kv["NVIC.TIM23_IRQn"])

    # ---- 4. 移除 USB_DEVICE 中间件 ----
    for k in [k for k in kv if k.startswith("USB_DEVICE.") or k.startswith("VP_USB_DEVICE")]:
        del kv[k]
    # 去掉中间件后 CubeMX 会丢弃原 OTG_HS 中断配置，需显式写入（取值同 QDU bsp-dev-mc02）
    kv["NVIC.OTG_HS_IRQn"] = "true\\:5\\:0\\:false\\:false\\:true\\:true\\:true\\:true\\:true"

    # ---- 5. FreeRTOS 任务与堆 ----
    # 去掉官方例程的演示任务（Key/Lcd/Imu/FunTest），只留 defaultTask 跑 LibXR app_main。
    # app_main 在该任务栈上构造全部外设对象，栈给 4096 字（16 KB，同 QDU bsp-dev-mc02）。
    kv["FREERTOS.Tasks01"] = "defaultTask,0,4096,StartDefaultTask,Default,NULL,Dynamic,NULL,NULL"
    kv["FREERTOS.configTOTAL_HEAP_SIZE"] = FREERTOS_HEAP
    params = kv["FREERTOS.IPParameters"].split(",")
    if "configTOTAL_HEAP_SIZE" not in params:
        params.append("configTOTAL_HEAP_SIZE")
    kv["FREERTOS.IPParameters"] = ",".join(params)

    # ---- IP / 虚拟引脚列表重编号 ----
    ips = [kv[k] for k in sorted((k for k in kv if re.match(r"^Mcu\.IP\d+$", k)),
                                 key=lambda k: int(k[6:]))]
    ips = [ip for ip in ips if ip != "USB_DEVICE"]
    if "BDMA" not in ips:
        ips.append("BDMA")
    for k in [k for k in kv if re.match(r"^Mcu\.IP\d+$", k)]:
        del kv[k]
    for i, ip in enumerate(ips):
        kv[f"Mcu.IP{i}"] = ip
    kv["Mcu.IPNb"] = str(len(ips))

    pins = [kv[k] for k in sorted((k for k in kv if re.match(r"^Mcu\.Pin\d+$", k)),
                                  key=lambda k: int(k[7:]))]
    pins = [p for p in pins if not p.startswith("VP_USB_DEVICE")]
    for k in [k for k in kv if re.match(r"^Mcu\.Pin\d+$", k)]:
        del kv[k]
    for i, p in enumerate(pins):
        kv[f"Mcu.Pin{i}"] = p
    kv["Mcu.PinsNb"] = str(len(pins))

    # 输出：保留注释头，键按字典序（CubeMX 自身保存格式）
    header = [line for k, line in order if k is None and line.startswith("#")]
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        for line in header:
            f.write(line + "\n")
        for k in sorted(kv):
            f.write(f"{k}={kv[k]}\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
