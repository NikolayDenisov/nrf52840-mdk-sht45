#include "driver_sht4x.h"
#include "nrf.h"
#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "nrf_delay.h"
#include "uart.h"

#define SCL_PIN 6
#define SDA_PIN 8
#define SHT4X_ADDR SHT4X_ADDRESS_0

static uint8_t iic_init(void) {

  // Конфигурация пинов SDA и SCL (подставьте свои номера пинов)
  NRF_P0->PIN_CNF[SDA_PIN] =
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | // SDA
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

  NRF_P0->PIN_CNF[SCL_PIN] =
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | // SCL
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
  NRF_TWIM0->PSEL.SCL = SCL_PIN;
  NRF_TWIM0->PSEL.SDA = SDA_PIN;
  NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400
                         << TWIM_FREQUENCY_FREQUENCY_Pos;
  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
  return 0;
}

static uint8_t iic_deinit(void) {
  NRF_TWIM0->ENABLE = 0;
  return 0;
}

static uint8_t iic_write_cmd(uint8_t addr, uint8_t *buf, uint8_t len) {
  NRF_TWIM0->ADDRESS = addr;
  NRF_TWIM0->TXD.PTR = (uint32_t)buf;
  NRF_TWIM0->TXD.MAXCNT = len;
  NRF_TWIM0->TXD.LIST = 0;
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;
  NRF_TWIM0->TASKS_STARTTX = 1;

  while (!NRF_TWIM0->EVENTS_LASTTX) {
  }
  NRF_TWIM0->EVENTS_LASTTX = 0;
  return 0;
}

static uint8_t iic_read_cmd(uint8_t addr, uint8_t *buf, uint8_t len) {
  NRF_TWIM0->ADDRESS = addr;
  NRF_TWIM0->RXD.PTR = (uint32_t)buf;
  NRF_TWIM0->RXD.MAXCNT = len;
  NRF_TWIM0->RXD.LIST = 0;
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTRX_STOP_Msk;
  NRF_TWIM0->TASKS_STARTRX = 1;

  while (!NRF_TWIM0->EVENTS_LASTRX) {
  }
  NRF_TWIM0->EVENTS_LASTRX = 0;
}

static void delay_ms(uint32_t ms) { nrf_delay_ms(ms); }

static void debug_print(const char *const fmt, ...) { uart_send_string(fmt); }

int main(void) {
  uart_init();
  uart_send_string("SHT45 Example!\r\n");

  sht4x_handle_t sht_handle;
  uint16_t temp_raw, hum_raw;
  float temp_c, hum_perc;

  DRIVER_SHT4X_LINK_INIT(&sht_handle, sht4x_handle_t);
  DRIVER_SHT4X_LINK_IIC_INIT(&sht_handle, iic_init);
  DRIVER_SHT4X_LINK_IIC_DEINIT(&sht_handle, iic_deinit);
  DRIVER_SHT4X_LINK_IIC_WRITE_COMMAND(&sht_handle, iic_write_cmd);
  DRIVER_SHT4X_LINK_IIC_READ_COMMAND(&sht_handle, iic_read_cmd);
  DRIVER_SHT4X_LINK_DELAY_MS(&sht_handle, delay_ms);
  DRIVER_SHT4X_LINK_DEBUG_PRINT(&sht_handle, debug_print);

  sht4x_set_addr(&sht_handle, SHT4X_ADDR);
  if (sht4x_init(&sht_handle) != 0) {
    debug_print("SHT45 init error\n");
    while (1)
      ;
  }
  while (1) {
    if (sht4x_read(&sht_handle, SHT4X_MODE_HIGH_PRECISION_WITH_NO_HEATER,
                   &temp_raw, &temp_c, &hum_raw, &hum_perc) == 0) {
      debug_print("T=%.2f °C, H=%.2f %%\n", temp_c, hum_perc);
    } else {
      debug_print("Read error\n");
    }
    nrf_delay_ms(1000);
  }
}
