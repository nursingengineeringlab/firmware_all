/**
 * @file       main.c
 * @version    1.0.0
 * @date       2022-09-25
 * @author     Abu Bony Amin, Ebenezer Asabre, Akshat Sahay 
 * @brief      EMG Signal Collection and Analysis 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include "nordic_common.h"
#include "app_error.h"
#include "boards.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "sys_bm.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_acs.h"
#include "ble_mgs.h"
#include "ble_gys.h"
#include "bsp_hw.h"
#include "bsp_imu.h"
#include "bsp_afe.h"
#include "bsp_nand_flash.h"
#include "nrf52832_peripherals.h"
#include "uart.h"
#include "bno085.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

/* Private defines ---------------------------------------------------- */
#define APP_BLE_CONN_CFG_TAG            1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define SENSORS_MEAS_INTERVAL           APP_TIMER_TICKS(2000)                      /**< Sensors measurement interval (ticks). */
#define BATT_LEVEL_MEAS_INTERVAL        APP_TIMER_TICKS(20000)                     /**< Battery level measurement interval (ticks). */

#define DEVICE_NAME                     "imu-lcd"                                  /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "miBEAT"                                   /**< Manufacturer. Will be passed to Device Information Service. */

#define ACS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

// custom
#define LED_STATE_INTERVAL              APP_TIMER_TICKS(2000)                       /** led interval **/



#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEVICE_ID                       0x55AA

#define SCALE_FACTOR_ACC                1000 
#define SCALE_FACTOR_GYR                10000
#define SCALE_FACTOR_MAG                1000 

#define TX_BYTE(x) do {} while (app_uart_put((x)) != NRF_SUCCESS)                   /**< Macro for UART TX. */ 

/* Private macros ----------------------------------------------------- */                                                            /**< BLE HRNS service instance. */
BLE_ACS_DEF(m_acs);                                                                 /**< BLE ACS service instance. */
BLE_MGS_DEF(m_mgs);                                                                 /**< BLE MGS service instance. */
BLE_GYS_DEF(m_gys);                                                                 /**< BLE GYS service instance. */
BLE_BAS_DEF(m_bas);                                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_sensors_timer_id);                                                  /**< Sensor measurement timer. */
APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
APP_TIMER_DEF(m_led_timer_id);                                                      /**< LED timer **/


/* Private variables -------------------------------------------------- */
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
  {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
  {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

uint32_t app_time;                                                                  /**< Elapsed time in the app. */                                                                                                                                           
int16_t emg_value;                                                                  /**< Current raw ECG/EMG sample from AFE. */  
int16_t imu[9];                                                                     /**< IMU data. */
uint8_t imu_size = 0;                                                               /**< Count of valid IMU data. */

uint8_t msec = 0;                                                                   /**< Millisecond counter synchronyzed to AFE interrupts. */

sh2_SensorValue_t sensor_value;

/* Private function prototypes ---------------------------------------- */
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void sleep_mode_enter(void);
static void log_init(void);
static void power_management_init(void);
static void idle_state_handle(void);

static void battery_level_meas_timeout_handler(void * p_context);
static void sensors_meas_timeout_handler(void * p_context);

static void battery_level_update(void);
static void u8p_send(uint8_t field, uint16_t value);
void setReports(void);  


// customer prototypes
static void timers_init(void);
static void application_timers_start(void);


// get LED
#define LED1  17
#define PIN_24  24

// put LED off after few seconds

void led_on()
{
  /** custom LED**/

  nrf_gpio_pin_set(LED1); // set logic 1 on pin 17
  //nrf_delay_ms(500);
 // printf("state: %d\n", nrf_gpio_pin_read(PIN_24));


}

/* Function definitions ----------------------------------------------- */

/**
 * @brief Application main function.
 */
int main(void)
{ 

  // g_device.is_device_on = false;


  // use timer and toggle toggle state of LED
  // custom
  timers_init();
  //nrf_gpio_cfg_output(LED1);
  //nrf_gpio_pin_set(LED1);  // set led to logic 1



  // initialize modules
  user_uart_init(); 
  power_management_init();

  // bsp_hw_init configures DRDY as an input 
  bsp_hw_init();
  bsp_afe_init();

  // initialize BNO085 over I2C
  bno085_init();

  // set default mode to record data
  // g_device.mode = SYS_MODE_RECORD_DATA;
  setReports();


  // custom
  application_timers_start();

  uint8_t imu_flags = 0;
  


  // enter MCU superloop
  for (;;) { 

  //led_on(); // put on LED


    if (bsp_afe_get_ecg(&emg_value) == BS_OK) {
      if (semaphore != 0) {
        // add bias and print 
        uint16_t emg_data = emg_value;  
        u8p_send(0, emg_value); 

        // sample IMU at 50 Hz = 1/20 sps 
        if (msec % 20 == 0) {
          // check if IMU was reset 
          if (bno085_was_reset()) setReports();

          // get data from IMU
          if (bno085_get_sensor_event(&sensor_value))
          {
            switch (sensor_value.sensorId)
            {
              case SH2_ACCELEROMETER:
                imu[0] = sensor_value.un.accelerometer.x * SCALE_FACTOR_ACC;
                imu[1] = sensor_value.un.accelerometer.y * SCALE_FACTOR_ACC;
                imu[2] = sensor_value.un.accelerometer.z * SCALE_FACTOR_ACC;
                imu_flags |= 0b001;
                break; 
              case SH2_GYROSCOPE_CALIBRATED:
                imu[3] = sensor_value.un.gyroscope.x * SCALE_FACTOR_GYR; 
                imu[4] = sensor_value.un.gyroscope.y * SCALE_FACTOR_GYR; 
                imu[5] = sensor_value.un.gyroscope.z * SCALE_FACTOR_GYR; 
                imu_flags |= 0b010;
                break;
              case SH2_MAGNETIC_FIELD_CALIBRATED:
                imu[6] = sensor_value.un.magneticField.x * SCALE_FACTOR_MAG;
                imu[7] = sensor_value.un.magneticField.y * SCALE_FACTOR_MAG;
                imu[8] = sensor_value.un.magneticField.z * SCALE_FACTOR_MAG;
                imu_flags |= 0b100;
                break; 
              default: break;
            } 

            if (imu_flags == 0b111 && imu_size == 0) {
              imu_size = 10;
              imu_flags = 0;
            }
          }
        }
        // send IMU data if any.
        if (imu_size) {
          imu_size--;
          if (imu_size) u8p_send(imu_size, imu[imu_size - 1]);
          else u8p_send(31, DEVICE_ID);
        }

        // increment ms counter 
        msec++;  
        // decrease semaphore 
        semaphore--; 
      } 
    }


   
  }



//home/ebenezer/Developer/aquatic-emg-sensor-main_x/emg_sensor_nrf5_SDK/app/project/ble_peripheral/ble_app_template/main.c
}


/**
 * @brief         Function for sending UART characters
 *
 * @param[in]     field   Field identifier 
 * @param[in]     value   16-bit value for transmission  
 *
 * @attention     None
 *
 * @return        None
 */
void u8p_send(uint8_t field, uint16_t value) {
    // break 16-bit value into 6-6-4 groups 
    uint8_t bits[] = {
        (value >> 12) & 0xf,
        (value >> 6) & 0x3f,
        value & 0x3f
    };

    // break field ID into 2-3
    uint8_t fields[] = {
        (field >> 2) & 0x7,
        (field & 0x3) << 4
    };

    // identifier for a byte in the middle of sequence 
    static const uint8_t continue_byte = 0x80;

    static const uint8_t two_prefix = 0xc0;

    // identifier for EMG data header 
    static const uint8_t three_prefix = 0xe0;

    // identifier for other data header 
    static const uint8_t four_prefix = 0xf0;
    
    if (field) {
        // if field != 0, send 4-byte sequence, if not sending EMG
        TX_BYTE(four_prefix | fields[0]);
        TX_BYTE(continue_byte | fields[1] | bits[0]);
        TX_BYTE(continue_byte | bits[1]);
        TX_BYTE(continue_byte | bits[2]);
    } 
    else if ((value & ~0x7f) == 0) {
        // value of EMG is less than 127  
        TX_BYTE(value); 
    }
    else if ((value & ~0x7ff) == 0) {
        // value of EMG is between 128 and 2047 
        TX_BYTE(two_prefix | bits[1]);
        TX_BYTE(continue_byte | bits[2]);
    }
    else {
        // if field == 0, send 3-byte sequence, if sending EMG
        TX_BYTE(three_prefix | bits[0]);
        TX_BYTE(continue_byte | bits[1]);
        TX_BYTE(continue_byte | bits[2]);
    }
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  // NRF_LOG_INFO("Setting desired reports");
  static const sh2_SensorId_t sensors[] = {
    SH2_ACCELEROMETER,
    SH2_GYROSCOPE_CALIBRATED,
    SH2_MAGNETIC_FIELD_CALIBRATED
  };

  for (int i = 0; i < sizeof(sensors) / sizeof(sh2_SensorId_t); i++) {
    sh2_SensorId_t sensor = sensors[i];

    if (!bno085_enable_report(sensor, 5000)) {
      // NRF_LOG_INFO("Could not enable accelerometer");
      u8p_send(30, 0xE000 | sensor); 
    }
  }
}

/**
 * @brief         Function for assert macro callback.
 *
 * @param[in]     line_num     Line number of the failing ASSERT call.
 * @param[in]     p_file_name  File name of the failing ASSERT call.
 *
 * @attention     None
 *
 * @return        None
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief         Function for handling Queued Write Module errors.
 *
 * @param[in]     nrf_error   Error code containing information about what went wrong.
 *
 * @attention     None
 *
 * @return        None
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief         Function for putting the chip into sleep mode.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void sleep_mode_enter(void)
{
  uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling events from the BSP module.
 *
 * @param[in]     event   Event generated by button press.
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_event_handler(bsp_event_t event)
{
  uint32_t err_code;
  switch (event)
  {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  default:
    break;
  }
}

/**
 * @brief         Function for initializing the nrf log module.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief         Function for initializing power management.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling the idle state (main loop).
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void idle_state_handle(void)
{
  if (NRF_LOG_PROCESS() == false)
  {
    nrf_pwr_mgmt_run();
  }
}

/**
 * @brief         Function for handling the Battery measurement timer timeout.
 *
 * @param[in]     p_context   Pointer to context
 *
 * @attention     None
 *
 * @return        None
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  battery_level_update();
}

/**
 * @brief         Function for handling the Body temperature measurement timer timeout.
 *
 * @param[in]     p_context   Pointer to context
 *
 * @attention     None
 *
 * @return        None
 */
static void sensors_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  // sensors_value_update();
}

/**
 * @brief         Function for handling the battery level update
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void battery_level_update(void)
{
  uint8_t battery_level              = 0;
  static  uint8_t battery_cal_time   = 0;
  static  uint16_t sum_battery_level = 0;

  sys_bm_get_level_in_percent(&battery_level);
  sum_battery_level += battery_level;
  battery_cal_time ++;

  if (battery_cal_time >= 10)
  {
    battery_level = sum_battery_level / battery_cal_time;
    // NRF_LOG_INFO( "Battery avg : %d percent", battery_level);
    battery_cal_time  = 0;
    sum_battery_level = 0;

    // ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
  }
}

//home/ebenezer/Developer//aquatic-emg-sensor-main_x/emg_sensor_nrf5_SDK/app/project/ble_peripheral/ble_app_template/main.c

/** LED timeout handler **/
static void led_timeout_handler(void * p_contex)
{

  // every interval toggle state of led
  nrf_gpio_pin_toggle(LED1);
  //printf("Hello");

}

/**@brief Function for starting applicatin timers

*/
static void timers_init(void)
{
  // initialize timer module
 // ret_code_t err_code;

  //err_code = app_timer_init();
 // APP_ERROR_CHECK(err_code);

  // Create timers.
  //err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_REPEATED, led_timeout_handler);
  //APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
  //ret_code_t err_code;

  //err_code = app_timer_start(m_led_timer_id, LED_STATE_INTERVAL, NULL);
 // APP_ERROR_CHECK(err_code);
}

/* End of file -------------------------------------------------------- */
