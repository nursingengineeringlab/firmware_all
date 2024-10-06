/**
 * @file       ads1292.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-07-31
 * @author     Bony
 * @brief      Driver support ADS1292 (Analog Front-End for Biopotential Measurements)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "ads1292r.h"

/* Private defines ---------------------------------------------------- */
#define ADS1292_ID_ADS1292R                 (0x73)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static unsigned char ads1293_id;

volatile int semaphore = 0;

// buffer for storing data from AFE in interrupt handler 
volatile char *spi_rx_buf_pointer;

/* Private function prototypes ---------------------------------------- */
static void ads1292_reset(const int pwdn_pin);
static void ads1292_spi_command_data(unsigned char data_in, const int chip_select);
static void ads1292_disable_start(const int start_pin);
static void ads1292_enable_start(const int start_pin);
static void ads1292_hard_stop(const int start_pin);
static void ads1292_start_data_conv_command(const int chip_select);
static void ads1292_soft_stop(const int chip_select);
static void ads1292_start_read_data_continuous(const int chip_select);
static void ads1292_stop_read_data_continuous(const int chip_select);
static char *ads1292_read_data(const int chip_select);
static void ads1292_reg_read(unsigned char read_addr, unsigned char *data, const int chip_select);
static void ads1292_reg_write(unsigned char read_write_addr, unsigned char data, const int chip_select);
static void ads1292_gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action); 

/* Function definitions ----------------------------------------------- */
base_status_t ads1292_get_ecg_and_respiration_sample(const int data_ready, const int chip_select, ads1292_output_value_t *data_sample)
{
  long     status_byte               = 0;
  uint8_t  lead_status               = 0;
  signed   long secg_temp            = 0;
  unsigned long result_temp          = 0;
  unsigned long uecg_temp            = 0;
  volatile static int spi_rx_buf_cnt = 0;
  int      j                         = 0;

  volatile uint8_t spi_rx_buf[15];

  // interrupt handler gets data and puts in spi_rx_buf_pointer

  // Store the result data in array
  for (int i = 0; i < 9; i++)
  {
    spi_rx_buf[spi_rx_buf_cnt++] = *(spi_rx_buf_pointer + i);
  }

  // Data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
  for (int i = 3; i < 9; i += 3) 
  {
    uecg_temp = (unsigned long)(((unsigned long)spi_rx_buf[i + 0] << 16) | ((unsigned long)spi_rx_buf[i + 1] << 8) | (unsigned long)spi_rx_buf[i + 2]);
    uecg_temp = (unsigned long)(uecg_temp << 8);
    secg_temp = (signed long)(uecg_temp);
    secg_temp = (signed long)(secg_temp >> 8);

    // daq_vals[0] is Resp data and daq_vals[1] is ECG data
    (data_sample->daq_vals)[j++] = secg_temp;
  }

  // First 3 bytes represents the status
  status_byte = (long)((long)spi_rx_buf[2] | ((long)spi_rx_buf[1]) << 8 | ((long)spi_rx_buf[0]) << 16);

  // Bit 15 gives the lead status
  status_byte = (status_byte & 0x0f8000) >> 15;
  lead_status = (unsigned char)status_byte;

  // 6,7,8
  result_temp = (uint32_t)((0 << 24) | (spi_rx_buf[3] << 16) | spi_rx_buf[4] << 8 | spi_rx_buf[5]);
  result_temp = (uint32_t)(result_temp << 8);

  data_sample->result_temp_resp = (long)(result_temp);

  // Check lead off detection
  if (!((lead_status & 0x1f) == 0))
    data_sample->lead_off_detected = true;
  else
    data_sample->lead_off_detected = false;

  spi_rx_buf_cnt = 0;

  return BS_OK;
}

base_status_t ads1292_init(const int chip_select, const int pwdn_pin, const int start_pin)
{
  ads1292_reset(pwdn_pin);
  platform_delay(100);

  ads1292_disable_start(start_pin);
  ads1292_enable_start(start_pin);
  ads1292_hard_stop(start_pin);
  ads1292_start_data_conv_command(chip_select);

  ads1292_soft_stop(chip_select);
  platform_delay(50);

  ads1292_stop_read_data_continuous(chip_select);
  platform_delay(300);

  // Check device ID
  ads1292_reg_read(ADS1292_REG_ID, &ads1293_id, chip_select);
  if (ads1293_id != ADS1292_ID_ADS1292R)
  {
    return BS_ERROR;
  } 

  // Set sampling rate to 1000 SPS
  // change to 010/0x02 for 500 SPS, 011/0x03 for 1000 SPS
  ads1292_reg_write(ADS1292_REG_CONFIG1, 0x03, chip_select);  
  platform_delay(10);

  // Lead-off comp off, test signal disabled
  ads1292_reg_write(ADS1292_REG_CONFIG2, 160, chip_select);   
  platform_delay(10);

  // Lead-off defaults
  ads1292_reg_write(ADS1292_REG_LOFF, 16, chip_select);       
  platform_delay(10);

  // Ch 1 enabled, gain 6, connected to electrode in
  ads1292_reg_write(ADS1292_REG_CH1SET, 64, chip_select);     
  platform_delay(10);

  // Ch 2 enabled, gain 6, connected to electrode in
  ads1292_reg_write(ADS1292_REG_CH2SET, 96, chip_select);     
  platform_delay(10);

  // RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  ads1292_reg_write(ADS1292_REG_RLDSENS, 44, chip_select);    
  platform_delay(10);

  // LOFF settings: all disabled
  // Skip register 8, LOFF Settings default
  ads1292_reg_write(ADS1292_REG_LOFFSENS, 0x00, chip_select); 
  platform_delay(10);                                         

  // Respiration: MOD/DEMOD turned only, phase 0
  ads1292_reg_write(ADS1292_REG_RESP1, 242, chip_select);     
  platform_delay(10);

  // Respiration: Calib OFF, respiration freq defaults
  ads1292_reg_write(ADS1292_REG_RESP2, 3, chip_select);       
  platform_delay(10);

  ads1292_start_read_data_continuous(chip_select);
  platform_delay(10);

  ads1292_enable_start(start_pin);

  // configure gpiote to detect FALLING edges 
  nrf_drv_gpiote_in_config_t  in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true); 
  in_config.pull = NRF_GPIO_PIN_PULLUP; 

  // configure DRDY to listen for falling edges 
  APP_ERROR_CHECK(nrf_drv_gpiote_in_init(IO_AFE_DRDY, &in_config, ads1292_gpiote_handler)); 

  // enable interrupts on DRDY
  nrf_drv_gpiote_in_event_enable(IO_AFE_DRDY, true); 

  return BS_OK;
}

/* Private function definitions --------------------------------------- */
static char *ads1292_read_data(const int chip_select)
{
  static char spi_dummy_buf[10];

  platform_write_pin(chip_select, false);

  for (int i = 0; i < 9; ++i)
    spi_dummy_buf[i] = platform_spi_transfer(CONFIG_SPI_MASTER_DUMMY);

  platform_write_pin(chip_select, true);

  return spi_dummy_buf;
}

static void ads1292_spi_command_data(unsigned char data_in,const int chip_select)
{
  platform_write_pin(chip_select, false);
  platform_delay(2);

  platform_write_pin(chip_select, true);
  platform_delay(2);

  platform_write_pin(chip_select, false);
  platform_delay(2);

  platform_spi_transfer(data_in);
  platform_delay(2);

  platform_write_pin(chip_select, true);
}

static void ads1292_reg_write (unsigned char read_write_addr, unsigned char data,const int chip_select)
{
  switch (read_write_addr)
  {
  case 1:
    data = data & 0x87;
    break;
  case 2:
    data = data & 0xFB;
    data |= 0x80;
    break;
  case 3:
    data = data & 0xFD;
    data |= 0x10;
    break;
  case 7:
    data = data & 0x3F;
    break;
  case 8:
    data = data & 0x5F;
    break;
  case 9:
    data |= 0x02;
    break;
  case 10:
    data = data & 0x87;
    data |= 0x01;
    break;
  case 11:
    data = data & 0x0F;
    break;
  default:
    break;
  }

  // Now combine the register address and the command into one uint8_t:
  uint8_t data_to_send = read_write_addr | ADS1292_CMD_WREG;
  platform_write_pin(chip_select, false);
  platform_delay(2);
  platform_write_pin(chip_select, true);
  platform_delay(2);

  // Take the chip select low to select the device:
  platform_write_pin(chip_select, false);
  platform_delay(2);
  platform_spi_transfer(data_to_send); // Send register location
  platform_spi_transfer(0x00);         // Number of register to wr
  platform_spi_transfer(data);         // Send value to record into register
  platform_delay(2);

  // Take the chip select high to de-select:
  platform_write_pin(chip_select, true);
}

static void ads1292_reg_read(unsigned char read_addr, unsigned char *data, const int chip_select)
{
  uint8_t data_to_send = read_addr | ADS1292_CMD_RREG;

  platform_write_pin(chip_select, false);
  platform_delay(2);

  platform_write_pin(chip_select, true);
  platform_delay(2);

  // Take the chip select low to select the device:
  platform_write_pin(chip_select, false);
  platform_delay(2);

  // Send register location
  platform_spi_transfer(data_to_send); 
  // Number of register to wr
  platform_spi_transfer(0x00);         

  // Send value to record into register
  *data = platform_spi_transfer(0x00); 
  platform_delay(2);

  // Take the chip select high to de-select:
  platform_write_pin(chip_select, true);
}

static void ads1292_reset(const int pwdn_pin)
{
  platform_write_pin(pwdn_pin, true);
  platform_delay(100);
  platform_write_pin(pwdn_pin, false);
  platform_delay(100);
  platform_write_pin(pwdn_pin, true);
  platform_delay(100);
}

static void ads1292_disable_start(const int start_pin)
{
  platform_write_pin(start_pin, false);
  platform_delay(20);
}

static void ads1292_enable_start(const int start_pin)
{
  platform_write_pin(start_pin, true);
  platform_delay(20);
}

static void ads1292_hard_stop (const int start_pin)
{
  platform_write_pin(start_pin, false);
  platform_delay(100);
}

static void ads1292_start_data_conv_command (const int chip_select)
{
  // Send 0x08 to the ADS1x9x
  ads1292_spi_command_data(ADS1292_CMD_START, chip_select); 
}

static void ads1292_soft_stop (const int chip_select)
{
  // Send 0x0A to the ADS1x9x
  ads1292_spi_command_data(ADS1292_CMD_STOP, chip_select); 
}

static void ads1292_start_read_data_continuous (const int chip_select)
{
  // Send 0x10 to the ADS1x9x
  ads1292_spi_command_data(ADS1292_CMD_RDATAC, chip_select); 
}

static void ads1292_stop_read_data_continuous (const int chip_select)
{
  // Send 0x11 to the ADS1x9x
  ads1292_spi_command_data(ADS1292_CMD_SDATAC, chip_select); 
}

static void ads1292_gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
  // Read the data, point the data to a pointer
  spi_rx_buf_pointer = ads1292_read_data(IO_AFE_CS); 

  // increase semaphore 
  semaphore++; 
}

/* End of file -------------------------------------------------------- */
