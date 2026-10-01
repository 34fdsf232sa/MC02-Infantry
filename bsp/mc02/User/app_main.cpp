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
  PlatformInit(2, 1024);
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
  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}