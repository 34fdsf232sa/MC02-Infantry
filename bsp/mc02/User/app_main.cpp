#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"

using namespace LibXR;

/* User Code Begin 1 */
#include <cstdlib>
#include <cstring>

#include <string_view>

#include "drivers/can_monitor.hpp"
#include "drivers/mit_motor.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

// 三路 FDCAN 的被动监视。只收不发：整个命令里没有任何发送路径。
namespace {
constexpr std::size_t kCanBuses = 3;
drivers::CanMonitor<kCanBuses, 16> g_can_monitor;

// 每条总线一个长期有效的上下文（回调里拿到的指针必须一直有效）
struct CanBusCtx {
  std::size_t bus;
};
CanBusCtx g_can_ctx[kCanBuses] = {{0}, {1}, {2}};

void OnCanFrame(bool /*in_isr*/, CanBusCtx* ctx, const LibXR::CAN::ClassicPack& p) {
  const bool ext = (p.type == LibXR::CAN::Type::EXTENDED);
  g_can_monitor.OnFrame(ctx->bus, p.id, ext, p.data, p.dlc <= 8 ? p.dlc : 8,
                        static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()));
}

// DJI C620/C610 反馈帧（标准帧 0x201..0x208）：
//   [0:1] 转子角度 0..8191 | [2:3] 转速 rpm (int16) | [4:5] 转矩电流 (int16) | [6] 温度 °C
bool LooksLikeDjiFeedback(std::uint32_t id, bool ext) { return !ext && id >= 0x201 && id <= 0x208; }

int CanCommand(int* /*ctx*/, int argc, char** argv) {
  using LibXR::STDIO;
  if (argc >= 2 && std::strcmp(argv[1], "clear") == 0) {
    g_can_monitor.Reset();
    STDIO::Printf<"CAN 统计已清零\r\n">();
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "stat") != 0) {
    STDIO::Printf<"用法: can [stat|clear]   (只收不发)\r\n">();
    return 1;
  }

  const std::uint32_t now = static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
  for (std::size_t bus = 0; bus < kCanBuses; ++bus) {
    STDIO::Printf<"FDCAN%u: 共 %u 帧, %u 个 ID, 溢出 %u\r\n">(
        static_cast<unsigned>(bus + 1), static_cast<unsigned>(g_can_monitor.Total(bus)),
        static_cast<unsigned>(g_can_monitor.Used(bus)),
        static_cast<unsigned>(g_can_monitor.Overflow(bus)));
    for (std::size_t i = 0; i < g_can_monitor.Used(bus); ++i) {
      drivers::CanFrameRecord r;
      if (!g_can_monitor.Snapshot(bus, i, r)) {
        continue;
      }
      const std::uint32_t span = r.last_ms - r.first_ms;
      const unsigned hz = span > 0 ? static_cast<unsigned>((r.count - 1) * 1000ull / span) : 0;
      const std::uint32_t age = now - r.last_ms;
      STDIO::Printf<"  %s0x%03X  n=%-7u ~%4uHz  距今%ums  %02X %02X %02X %02X %02X %02X %02X %02X\r\n">(
          r.extended ? "E:" : "  ", static_cast<unsigned>(r.id), static_cast<unsigned>(r.count), hz,
          static_cast<unsigned>(age), r.data[0], r.data[1], r.data[2], r.data[3], r.data[4],
          r.data[5], r.data[6], r.data[7]);
      if (LooksLikeDjiFeedback(r.id, r.extended)) {
        const auto ang = static_cast<unsigned>((r.data[0] << 8) | r.data[1]);
        const auto rpm = static_cast<int16_t>((r.data[2] << 8) | r.data[3]);
        const auto cur = static_cast<int16_t>((r.data[4] << 8) | r.data[5]);
        STDIO::Printf<"        -> 电调ID %u: 角度 %u/8191 (%u.%u°)  转速 %d rpm  转矩电流 %d  温度 %u°C\r\n">(
            static_cast<unsigned>(r.id - 0x200), ang, ang * 360u / 8192u, (ang * 3600u / 8192u) % 10u,
            static_cast<int>(rpm), static_cast<int>(cur), static_cast<unsigned>(r.data[6]));
      }
    }
  }
  return 0;
}

struct Send0Context {
  LibXR::CAN* can[3];
};

// send0 [1|2|3]：往指定 FDCAN 发一帧 0x200 全零（零电流），缺省为 1
int Send0Command(Send0Context* ctx, int argc, char** argv) {
  using LibXR::STDIO;
  int bus = 1;
  if (argc >= 2) {
    bus = argv[1][0] - '0';
  }
  if (bus < 1 || bus > 3) {
    STDIO::Printf<"用法: send0 [1|2|3]\r\n">();
    return 1;
  }
  LibXR::CAN::ClassicPack pack{};
  pack.id = 0x200;
  pack.type = LibXR::CAN::Type::STANDARD;
  pack.dlc = 8;
  for (auto& byte : pack.data) {
    byte = 0;
  }
  const auto ret = ctx->can[bus - 1]->AddMessage(pack);
  if (ret == LibXR::ErrorCode::OK) {
    STDIO::Printf<"已发送 0x200 [00 00 00 00 00 00 00 00] 到 FDCAN%d\r\n">(bus);
  } else {
    STDIO::Printf<"AddMessage 失败，错误码 %d\r\n">(static_cast<int>(ret));
  }
  return (ret == LibXR::ErrorCode::OK) ? 0 : 1;
}

bool SendDji200(LibXR::CAN* can, int16_t cur, int esc_id = 2) {
  LibXR::CAN::ClassicPack pack{};
  pack.id = 0x200;
  pack.type = LibXR::CAN::Type::STANDARD;
  pack.dlc = 8;
  for (auto& byte : pack.data) {
    byte = 0;
  }
  const int slot = (esc_id - 1) * 2;
  pack.data[slot] = static_cast<uint8_t>(static_cast<uint16_t>(cur) >> 8);
  pack.data[slot + 1] = static_cast<uint8_t>(static_cast<uint16_t>(cur) & 0xFF);
  return can->AddMessage(pack) == LibXR::ErrorCode::OK;
}

// 按电调 ID 在 FDCAN1 上以 1kHz 重复发送 0x200 电流帧，结束后回零并连发 20 帧零电流，
// 期间读取 0x200+ID 的反馈统计转速。m3508 与 m2006 各自有独立命令和固定参数。
int DjiRun(Send0Context* ctx, int cur, int ms, int esc_id) {
  using LibXR::STDIO;
  int16_t rpm_max = 0;
  int16_t rpm_last = 0;
  for (int t = 0; t < ms; ++t) {
    SendDji200(ctx->can[0], static_cast<int16_t>(cur), esc_id);
    LibXR::Thread::Sleep(1);
    if (t % 20 == 0) {
      for (std::size_t i = 0; i < g_can_monitor.Used(0); ++i) {
        drivers::CanFrameRecord r;
        if (g_can_monitor.Snapshot(0, i, r) && !r.extended &&
            r.id == static_cast<uint32_t>(0x200 + esc_id)) {
          rpm_last = static_cast<int16_t>((r.data[2] << 8) | r.data[3]);
          if (rpm_last > rpm_max) rpm_max = rpm_last;
          if (-rpm_last > rpm_max) rpm_max = static_cast<int16_t>(-rpm_last);
        }
      }
    }
  }
  for (int t = 0; t < 20; ++t) {
    SendDji200(ctx->can[0], 0, esc_id);
    LibXR::Thread::Sleep(1);
  }
  STDIO::Printf<"完成: 电调ID %d 电流 %d 持续 %d ms, 期间最大|转速| %d rpm, 末次 %d rpm, 已回零\r\n">(
      esc_id, cur, ms, static_cast<int>(rpm_max), static_cast<int>(rpm_last));
  return 0;
}

// m3508 <电流> <毫秒>：C620 + M3508，电调 ID 固定为 2，限幅 |电流|<=1000、时间<=2000ms。
int M3508Command(Send0Context* ctx, int argc, char** argv) {
  using LibXR::STDIO;
  if (argc < 3) {
    STDIO::Printf<"用法: m3508 <电流 -1000..1000> <时间ms 1..2000>   (电调ID固定为2)\r\n">();
    return 1;
  }
  int cur = std::atoi(argv[1]);
  int ms = std::atoi(argv[2]);
  if (cur < -1000 || cur > 1000 || ms < 1 || ms > 2000) {
    STDIO::Printf<"参数越界\r\n">();
    return 1;
  }
  return DjiRun(ctx, cur, ms, 2);
}

// m2006 <电调ID 1..4> <电流> <毫秒>：C610 + M2006，电调 ID 必须显式给出，
// 限幅 |电流|<=600、时间<=2000ms。
int M2006Command(Send0Context* ctx, int argc, char** argv) {
  using LibXR::STDIO;
  if (argc < 4) {
    STDIO::Printf<"用法: m2006 <电调ID 1..4> <电流 -600..600> <时间ms 1..2000>\r\n">();
    return 1;
  }
  int esc_id = std::atoi(argv[1]);
  int cur = std::atoi(argv[2]);
  int ms = std::atoi(argv[3]);
  if (esc_id < 1 || esc_id > 4 || cur < -600 || cur > 600 || ms < 1 || ms > 2000) {
    STDIO::Printf<"参数越界\r\n">();
    return 1;
  }
  return DjiRun(ctx, cur, ms, esc_id);
}

// ---- DM4310（MIT 模式），FDCAN1，假定出厂量程 PMAX 12.5 / VMAX 30 / TMAX 10，反馈 ID（Master ID）= 0 ----
constexpr drivers::MitRange kDmRange{12.5f, 30.0f, 10.0f};

bool SendStd(LibXR::CAN* can, uint32_t id, const drivers::CanPayload& d) {
  LibXR::CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = LibXR::CAN::Type::STANDARD;
  pack.dlc = 8;
  for (int i = 0; i < 8; ++i) {
    pack.data[i] = d[i];
  }
  return can->AddMessage(pack) == LibXR::ErrorCode::OK;
}

void PrintDmFeedback() {
  using LibXR::STDIO;
  for (std::size_t i = 0; i < g_can_monitor.Used(0); ++i) {
    drivers::CanFrameRecord r;
    if (!g_can_monitor.Snapshot(0, i, r) || r.extended || r.id != 0x000) {
      continue;
    }
    drivers::CanPayload d;
    for (int k = 0; k < 8; ++k) d[k] = r.data[k];
    drivers::MitFeedback fb;
    if (drivers::DecodeFeedback(d, kDmRange, fb)) {
      STDIO::Printf<"  反馈 0x000: 电机ID %u 状态 0x%X 位置 %d mrad 速度 %d mrad/s 力矩 %d mNm MOS %d°C 转子 %d°C\r\n">(
          static_cast<unsigned>(fb.motor_id), static_cast<unsigned>(fb.state),
          static_cast<int>(fb.position * 1000), static_cast<int>(fb.velocity * 1000),
          static_cast<int>(fb.torque * 1000), static_cast<int>(fb.mos_temp),
          static_cast<int>(fb.rotor_temp));
      return;
    }
  }
  STDIO::Printf<"  未收到 ID 0x000 的反馈\r\n">();
}

// 运行时改 FDCAN1 位速率（内核 120MHz，15 tq/bit，采样点 80%）。仅支持 125/250/500/1000 kbps。
bool SetCan1Kbps(int kbps) {
  uint32_t presc = 0, seg1 = 11, seg2 = 3, sjw = 3;
  if (kbps == 125 || kbps == 250 || kbps == 500 || kbps == 1000) {
    presc = 8u * (1000u / static_cast<uint32_t>(kbps));
  } else if (kbps == 2000) {
    presc = 4;
  } else if (kbps == 4000) {  // 30 tq/bit，采样点 80%
    presc = 1; seg1 = 23; seg2 = 6;
  } else if (kbps == 5000) {  // 24 tq/bit，采样点 83%
    presc = 1; seg1 = 19; seg2 = 4;
  } else {
    return false;
  }
  FDCAN_GlobalTypeDef* c = hfdcan1.Instance;
  SET_BIT(c->CCCR, FDCAN_CCCR_INIT);
  while ((c->CCCR & FDCAN_CCCR_INIT) == 0U) {
  }
  SET_BIT(c->CCCR, FDCAN_CCCR_CCE);
  c->NBTP = ((sjw - 1u) << FDCAN_NBTP_NSJW_Pos) | ((presc - 1u) << FDCAN_NBTP_NBRP_Pos) |
            ((seg1 - 1u) << FDCAN_NBTP_NTSEG1_Pos) | ((seg2 - 1u) << FDCAN_NBTP_NTSEG2_Pos);
  CLEAR_BIT(c->CCCR, FDCAN_CCCR_INIT);
  while ((c->CCCR & FDCAN_CCCR_INIT) != 0U) {
  }
  return true;
}

// dmscan：依次用 125/250/500/1000 kbps 向 ID 1..3 发失能帧，看 TEC 变化和回传帧数
int DmScanCommand(Send0Context* ctx, int /*argc*/, char** /*argv*/) {
  using LibXR::STDIO;
  const int rates[] = {2000, 4000, 5000};
  for (int kbps : rates) {
    SetCan1Kbps(kbps);
    LibXR::Thread::Sleep(20);
    g_can_monitor.Reset();
    const uint32_t tec0 = hfdcan1.Instance->ECR & 0xFFu;
    for (int id = 1; id <= 3; ++id) {
      SendStd(ctx->can[0], static_cast<uint32_t>(id), drivers::kDisableFrame);
      LibXR::Thread::Sleep(20);
    }
    const uint32_t tec1 = hfdcan1.Instance->ECR & 0xFFu;
    STDIO::Printf<"%4d kbps: TEC %u -> %u, 收到 %u 帧, PSR=0x%03X\r\n">(
        kbps, static_cast<unsigned>(tec0), static_cast<unsigned>(tec1),
        static_cast<unsigned>(g_can_monitor.Total(0)),
        static_cast<unsigned>(hfdcan1.Instance->PSR & 0x7FFu));
  }
  SetCan1Kbps(1000);
  STDIO::Printf<"已恢复 1000 kbps\r\n">();
  return 0;
}

// dm <id> dis|en|zero|clear|status
// dm <id> pos <mrad> <kp> <kd> <ms>   限幅 |位置|<=1000mrad kp<=20 kd<=2 ms<=2000，结束自动零增益
int DmCommand(Send0Context* ctx, int argc, char** argv) {
  using LibXR::STDIO;
  if (argc < 3) {
    STDIO::Printf<"用法: dm <id> dis|en|zero|clear|status | dm <id> pos <mrad> <kp> <kd> <ms>\r\n">();
    return 1;
  }
  const int id = std::atoi(argv[1]);
  if (id < 1 || id > 15) {
    STDIO::Printf<"id 越界\r\n">();
    return 1;
  }
  LibXR::CAN* can = ctx->can[0];
  const std::string_view sub = argv[2];
  g_can_monitor.Reset();
  if (sub == "dis" || sub == "en" || sub == "zero" || sub == "clear" || sub == "status") {
    const drivers::CanPayload& f = sub == "en"     ? drivers::kEnableFrame
                                   : sub == "zero" ? drivers::kSetZeroFrame
                                   : sub == "clear" ? drivers::kClearErrorFrame
                                                    : drivers::kDisableFrame;
    SendStd(can, static_cast<uint32_t>(id), f);
    LibXR::Thread::Sleep(50);
    PrintDmFeedback();
    return 0;
  }
  if (sub == "pos" && argc >= 7) {
    const int mrad = std::atoi(argv[3]);
    const float kp = static_cast<float>(std::atoi(argv[4]));
    const float kd = static_cast<float>(std::atoi(argv[5]));
    const int ms = std::atoi(argv[6]);
    if (mrad < -1000 || mrad > 1000 || kp < 0 || kp > 20 || kd < 0 || kd > 2 || ms < 1 || ms > 2000) {
      STDIO::Printf<"参数越界\r\n">();
      return 1;
    }
    drivers::MitCommand c;
    c.position = static_cast<float>(mrad) / 1000.0f;
    c.kp = kp;
    c.kd = kd;
    SendStd(can, static_cast<uint32_t>(id), drivers::kEnableFrame);
    LibXR::Thread::Sleep(20);
    for (int t = 0; t < ms; t += 5) {
      SendStd(can, static_cast<uint32_t>(id), drivers::EncodeCommand(c, kDmRange));
      LibXR::Thread::Sleep(5);
    }
    PrintDmFeedback();
    drivers::MitCommand z;
    for (int t = 0; t < 10; ++t) {
      SendStd(can, static_cast<uint32_t>(id), drivers::EncodeCommand(z, kDmRange));
      LibXR::Thread::Sleep(5);
    }
    SendStd(can, static_cast<uint32_t>(id), drivers::kDisableFrame);
    STDIO::Printf<"完成, 已零增益并失能\r\n">();
    return 0;
  }
  STDIO::Printf<"未知子命令\r\n">();
  return 1;
}
}  // namespace
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;

/* DMA Resources */
static uint16_t adc1_buf[32] __attribute__((section(".axi_ram")));
static uint8_t spi1_tx_buf[32] __attribute__((section(".axi_ram")));
static uint8_t spi2_tx_buf[32] __attribute__((section(".axi_ram")));
static uint8_t spi2_rx_buf[32] __attribute__((section(".axi_ram")));
static uint8_t spi6_tx_buf[32] __attribute__((section(".ram_d3")));
static uint8_t uart7_tx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t uart7_rx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart1_tx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart1_rx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart10_tx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart10_rx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart2_tx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart2_rx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart3_tx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usart3_rx_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usb_otg_hs_ep0_in_buf[8] __attribute__((section(".axi_ram")));
static uint8_t usb_otg_hs_ep0_out_buf[8] __attribute__((section(".axi_ram")));
static uint8_t usb_otg_hs_ep1_in_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usb_otg_hs_ep1_out_buf[128] __attribute__((section(".axi_ram")));
static uint8_t usb_otg_hs_ep2_in_buf[16] __attribute__((section(".axi_ram")));

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32TimerTimebase timebase(&htim23);
  PlatformInit(2, 4096);
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO PA15(GPIOA, GPIO_PIN_15);
  STM32GPIO LCD_BLK(LCD_BLK_GPIO_Port, LCD_BLK_Pin);
  STM32GPIO LCD_RES(LCD_RES_GPIO_Port, LCD_RES_Pin);
  STM32GPIO DCMI_REST(DCMI_REST_GPIO_Port, DCMI_REST_Pin);
  STM32GPIO ACC_CS(ACC_CS_GPIO_Port, ACC_CS_Pin);
  STM32GPIO POWER_24V_2(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin);
  STM32GPIO POWER_24V_1(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin);
  STM32GPIO POWER_5V(POWER_5V_GPIO_Port, POWER_5V_Pin);
  STM32GPIO GYRO_CS(GYRO_CS_GPIO_Port, GYRO_CS_Pin);
  STM32GPIO DCMI_PWDN(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin);
  STM32GPIO LCD_DC(LCD_DC_GPIO_Port, LCD_DC_Pin);
  STM32GPIO ACC_INT(ACC_INT_GPIO_Port, ACC_INT_Pin, EXTI15_10_IRQn);
  STM32GPIO GYRO_INT(GYRO_INT_GPIO_Port, GYRO_INT_Pin, EXTI15_10_IRQn);
  STM32GPIO LCD_CS(LCD_CS_GPIO_Port, LCD_CS_Pin);

  STM32ADC adc1(&hadc1, adc1_buf, {ADC_CHANNEL_4, ADC_CHANNEL_19}, 3.3);
  auto adc1_adc_channel_4 = adc1.GetChannel(0);
  UNUSED(adc1_adc_channel_4);
  auto adc1_adc_channel_19 = adc1.GetChannel(1);
  UNUSED(adc1_adc_channel_19);

  STM32PWM pwm_tim1_ch1(&htim1, TIM_CHANNEL_1, false);
  STM32PWM pwm_tim1_ch3(&htim1, TIM_CHANNEL_3, false);

  STM32PWM pwm_tim12_ch2(&htim12, TIM_CHANNEL_2, false);

  STM32PWM pwm_tim2_ch1(&htim2, TIM_CHANNEL_1, false);
  STM32PWM pwm_tim2_ch3(&htim2, TIM_CHANNEL_3, false);

  STM32PWM pwm_tim3_ch4(&htim3, TIM_CHANNEL_4, false);

  STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);

  STM32SPI spi2(&hspi2, spi2_rx_buf, spi2_tx_buf, 3);

  STM32SPI spi6(&hspi6, {nullptr, 0}, spi6_tx_buf, 3);

  STM32UART uart7(&huart7,
              uart7_rx_buf, uart7_tx_buf, 5);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32UART usart10(&huart10,
              usart10_rx_buf, usart10_tx_buf, 5);

  STM32UART usart2(&huart2,
              usart2_rx_buf, usart2_tx_buf, 5);

  STM32UART usart3(&huart3,
              usart3_rx_buf, usart3_tx_buf, 5);

  STM32CANFD fdcan1(&hfdcan1, 5);

  STM32CANFD fdcan2(&hfdcan2, 5);

  STM32CANFD fdcan3(&hfdcan3, 5);

  static constexpr auto USB_OTG_HS_LANG_PACK = LibXR::USB::DescriptorStrings::MakeLanguagePack(LibXR::USB::DescriptorStrings::Language::EN_US, "MC02-Infantry", "MC02", "MC02-0001");
  LibXR::USB::CDCUart usb_otg_hs_cdc(LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP2, 128, 128, 3);

  STM32USBDeviceOtgHS usb_hs(
      &hpcd_USB_OTG_HS,
      256,
      {usb_otg_hs_ep0_out_buf, usb_otg_hs_ep1_out_buf},
      {{usb_otg_hs_ep0_in_buf, 8}, {usb_otg_hs_ep1_in_buf, 128}, {usb_otg_hs_ep2_in_buf, 16}},
      USB::DeviceDescriptor::PacketSize0::SIZE_8,
      0x483, 0x5740, 0x100,
      {&USB_OTG_HS_LANG_PACK},
      {{&usb_otg_hs_cdc}},
      {reinterpret_cast<void *>(UID_BASE), 12}
  );
  usb_hs.Init(false);
  usb_hs.Start(false);

  /* Terminal Configuration */
  STDIO::read_ = usb_otg_hs_cdc.read_port_;
  STDIO::write_ = usb_otg_hs_cdc.write_port_;

  RamFS ramfs("XRobot");
  Terminal<32, 32, 5, 5> terminal(ramfs);
  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  // 被动监视三路 FDCAN：只注册接收回调，不发送任何帧。回调在中断里，只做计数和拷贝。
  static LibXR::CAN::Callback can_rx_cb[kCanBuses] = {
      LibXR::CAN::Callback::Create(OnCanFrame, &g_can_ctx[0]),
      LibXR::CAN::Callback::Create(OnCanFrame, &g_can_ctx[1]),
      LibXR::CAN::Callback::Create(OnCanFrame, &g_can_ctx[2]),
  };
  LibXR::CAN* const buses[kCanBuses] = {&fdcan1, &fdcan2, &fdcan3};
  for (std::size_t i = 0; i < kCanBuses; ++i) {
    buses[i]->Register(can_rx_cb[i], LibXR::CAN::Type::STANDARD);
    buses[i]->Register(can_rx_cb[i], LibXR::CAN::Type::EXTENDED);
  }

  static int can_cmd_arg = 0;
  static auto can_cmd = LibXR::RamFS::CreateCommand<int*>("can", CanCommand, &can_cmd_arg);
  ramfs.Add(can_cmd);

  static Send0Context send0_ctx{{&fdcan1, &fdcan2, &fdcan3}};
  static auto send0_cmd =
      LibXR::RamFS::CreateCommand<Send0Context*>("send0", Send0Command, &send0_ctx);
  ramfs.Add(send0_cmd);

  static auto m3508_cmd =
      LibXR::RamFS::CreateCommand<Send0Context*>("m3508", M3508Command, &send0_ctx);
  ramfs.Add(m3508_cmd);

  static auto m2006_cmd =
      LibXR::RamFS::CreateCommand<Send0Context*>("m2006", M2006Command, &send0_ctx);
  ramfs.Add(m2006_cmd);

  static auto dm_cmd = LibXR::RamFS::CreateCommand<Send0Context*>("dm", DmCommand, &send0_ctx);
  ramfs.Add(dm_cmd);

  static auto dmscan_cmd =
      LibXR::RamFS::CreateCommand<Send0Context*>("dmscan", DmScanCommand, &send0_ctx);
  ramfs.Add(dmscan_cmd);

  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}