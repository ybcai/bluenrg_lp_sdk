/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : AMS - RF Application team
 * Version            : V1.0.0
 * Date               : 02-April-2019
 * Description        : ST BLE Sensor/BlueMS Sensor init and sensor state machines
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"

#include "gatt_db.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_config.h"
#include "OTA_btl.h" 
#include "gap_profile.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define  ADV_INTERVAL_MIN_MS  1000
#define  ADV_INTERVAL_MAX_MS  1200

#define BLE_SENSOR_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;
uint32_t start_time = 0;
uint16_t connection_handle = 0;

int connected = FALSE;

static VTIMER_HandleType sensorTimerHandle;
static uint16_t acceleration_update_rate = 100;
static uint8_t sensorTimer_expired = FALSE;

#ifndef SENSOR_ACCELEROMETER_EMULATION

lsm6dsox_ctx_t inertialHandle;

#endif

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION
/* LPS22HH initialization */
lps22hh_ctx_t pressureHandle;

#endif 

volatile uint8_t request_free_fall_notify = FALSE; 
  
static uint8_t adv_data[] = {
    0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    10,AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','L','P', // Complete Name
    13,0xFF,0x01, /*SKD version */
    0x04, /* 0x04: BlueNRG1-1/2 eval kits */
    0x00,
    0xD4, /* ACC+Gyro 0xC0 | 0x04 Temp | 0x10 Pressure*/
    0x00, /*  */
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };
  
static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

/* Private function prototypes -----------------------------------------------*/
void SensorUpdateTimeoutCB(void *param);
/* Private functions ---------------------------------------------------------*/

#ifndef SENSOR_ACCELEROMETER_EMULATION
/*******************************************************************************
 * Function Name  : Init_Accelerometer.
 * Description    : Init LIS331DLH accelerometer.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Accelerometer(void)
{
  uint8_t rst;
  
  //lsm6dsox_pin_int1_route_t int_1_reg;
  /*
   * Uncomment if interrupt generation on Free Fall INT2 pin.
   */
  //lsm6dsox_int2_route_t int_2_reg;
  
  
  /* Initialize the handle of the LSM6DSO driver */
  inertialHandle.write_reg = BSP_SPI_Write;
  inertialHandle.read_reg = BSP_SPI_Read;
  
  /* Inizialize the SPI */
  BSP_SPI_Init();
  
  /* Restore default configuration */
  lsm6dsox_reset_set(&inertialHandle, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&inertialHandle, &rst);
  } while (rst);
  
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&inertialHandle, PROPERTY_ENABLE);
  
  /* Set full scale */  
  lsm6dsox_xl_full_scale_set(&inertialHandle, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&inertialHandle, LSM6DSOX_2000dps);
  
  /* Set Output Data Rate for Acc and Gyro */
  lsm6dsox_xl_data_rate_set(&inertialHandle, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&inertialHandle, LSM6DSOX_GY_ODR_12Hz5);
  
}

#endif 

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION
/*******************************************************************************
 * Function Name  : Init_Pressure_Temperature_Sensor.
 * Description    : Init LPS22HH pressure and temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Pressure_Temperature_Sensor(void)
{  
  /* LPS22HH initialization */
  uint8_t rst;
  
   /* Initialize the handle of the LPS22HH driver */
  pressureHandle.write_reg = BSP_I2C_Write;
  pressureHandle.read_reg = BSP_I2C_Read;
  
  /* Inizialize the SPI */
  BSP_I2C_Init();
  
  /* Restore default configuration */
  lps22hh_reset_set(&pressureHandle, PROPERTY_ENABLE);
  do {
    lps22hh_reset_get(&pressureHandle, &rst);
  } while (rst);
  
  /*  Enable Block Data Update */
  lps22hh_block_data_update_set(&pressureHandle, PROPERTY_ENABLE);
  
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&pressureHandle, LPS22HH_1_Hz_LOW_NOISE);
}
#endif 

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *******************************************************************************/
uint8_t Sensor_DeviceInit()
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G', 'L', 'P'};
  
  /* Set the TX power 0 dBm */
  aci_hal_set_tx_power_level(0, 25);
  
  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x09, STATIC_RANDOM_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
 
  /* Update device name */
   /* Update device name */
  ret = Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("Gap_profile_set_dev_name() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_NOT_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
    return ret;
  } 
  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              (ADV_INTERVAL_MIN_MS*1000)/625,(ADV_INTERVAL_MAX_MS*1000)/625,
                                              ADV_CH_ALL,
                                              STATIC_RANDOM_ADDR,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  PRINTF("Advertising configuration 0x%02X\n", ret);
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  PRINTF("Set advertising data 0x%02X\n", ret);  
  
  PRINTF("BLE Stack Initialized with SUCCESS\n");

#ifndef SENSOR_ACCELEROMETER_EMULATION /* User Real sensors */
  Init_Accelerometer();
#endif
  
#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION
  Init_Pressure_Temperature_Sensor();
#endif

  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
    PRINTF("BlueMS service added successfully.\n");
  } else {
    PRINTF("Error while adding BlueMS service: 0x%02x\r\n", ret);
    return ret;
  }
  
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT     
  ret = OTA_Add_Btl_Service();
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("OTA service added successfully.\n");
  else
    PRINTF("Error while adding OTA service.\n");
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 

  /* Start the Sensor Timer */
  sensorTimerHandle.callback = SensorUpdateTimeoutCB;  
  ret = HAL_VTIMER_StartTimerMs(&sensorTimerHandle, acceleration_update_rate);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("HAL_VTIMER_StartTimerMs() failed; 0x%02x\r\n", ret);
    return ret;
  } else {
    sensorTimer_expired = FALSE;
  }

  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{  
  uint8_t ret;
  
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  ret = aci_gap_set_scan_response_data(0,18,BTLServiceUUID4Scan);
  if(ret != BLE_STATUS_SUCCESS)
  {
    printf("aci_gap_set_scan_response_data() failed: 0x%02x\r\n",ret);
  }
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  //enable advertising
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_advertising_enable() failed: 0x%02x\r\n",ret);
    BSP_LED_On(BSP_LED3);  
  }
  else
    PRINTF("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
}


uint8_t GetPressure(float * pressure_hPa)
{
#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION /* User Real sensors */
   axis1bit32_t data_raw_pressure;
   lps22hh_reg_t reg;

   data_raw_pressure.i32bit = 0;
   
   lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
   if (reg.status.p_da) 
   {
      lps22hh_pressure_raw_get(&pressureHandle, data_raw_pressure.u8bit);
      *pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure.i32bit);
   }
   return (reg.status.p_da);
#else
   uint8_t tmp = 1;
   *pressure_hPa = 100 * (1000 + ((uint64_t)rand()*1000)/RAND_MAX);
   return (tmp);
#endif
}

/* Update temperature data in advertising packets */
uint8_t GetTemperature(float * temperature_degC)
{
#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION /* User Real sensors */
    axis1bit16_t data_raw_temperature;
    lps22hh_reg_t reg;

    /* Read output only if new value is available */
   lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
   if (reg.status.t_da) 
   {
      lps22hh_temperature_raw_get(&pressureHandle, data_raw_temperature.u8bit);
      *temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);
   }
   return (reg.status.t_da);
#else
    uint8_t tmp = 1;
    *temperature_degC = 450 + ((uint64_t)rand()*100)/RAND_MAX; 
    return (tmp);
#endif
}

uint8_t GetAccAxesRaw(AxesRaw_t * acceleration_data, AxesRaw_t * gyro_data)
{
#ifndef SENSOR_ACCELEROMETER_EMULATION /* User Real sensors */
  uint8_t tmp = 0;
  (void)tmp;
  
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
    
   /* Read output only if new value is available */
  lsm6dsox_xl_flag_data_ready_get(&inertialHandle, &tmp);
  if(tmp)
  {
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    /* Read acceleration field data */
    lsm6dsox_acceleration_raw_get(&inertialHandle, data_raw_acceleration.u8bit);
    acceleration_data->AXIS_X = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]); //TBR sm6ds3_from_fs2g_to_mg() returns a float
    acceleration_data->AXIS_Y = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration_data->AXIS_Z = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
  }
  
  lsm6dsox_gy_flag_data_ready_get(&inertialHandle, &tmp);
  if(tmp) {
    /* Read angular rate field data */
    lsm6dsox_angular_rate_raw_get(&inertialHandle, data_raw_angular_rate.u8bit);
    gyro_data->AXIS_X  = (int32_t)lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
    gyro_data->AXIS_Y  = (int32_t)lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
    gyro_data->AXIS_Z  = (int32_t)lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
  }
#else
  uint8_t tmp = 1;
  acceleration_data->AXIS_X = ((uint64_t)rand()) % X_OFFSET;
  acceleration_data->AXIS_Y = ((uint64_t)rand()) % Y_OFFSET; 
  acceleration_data->AXIS_Z = ((uint64_t)rand()) % Z_OFFSET;
  
#endif
      
  return(tmp);
}

/*******************************************************************************
 * Function Name  : APP_Tick.
 * Description    : Sensor Demo state machine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void APP_Tick(void)
{
  /* Make the device discoverable */
  if(set_connectable) {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }
    
  /*  Update sensor value */
  if (sensorTimer_expired) {
    sensorTimer_expired = FALSE;
    if( HAL_VTIMER_StartTimerMs(&sensorTimerHandle, acceleration_update_rate) != BLE_STATUS_SUCCESS)
      sensorTimer_expired = TRUE;
    if(connected) {
      AxesRaw_t x_axes, g_axes;
      float data_t, data_p;
      
      /* Activity Led */
      BSP_LED_Toggle(BSP_LED1);  

      /* Get Acceleration data */
      if (GetAccAxesRaw(&x_axes, &g_axes)) { 
        Acc_Update(&x_axes, &g_axes);
      }
      
      if (GetTemperature(&data_t) == 1) 
      {
        GetPressure(&data_p);
        BlueMS_Environmental_Update((int32_t)(data_p *100), (int16_t)(data_t * 10));
      }      
    }
  }

}

/* ***************** BlueNRG-LP Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  connected = TRUE;
  start_time = HAL_VTIMER_GetCurrentSysTime();
  connection_handle = Connection_Handle;
    
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_le_enhanced_connection_complete_event.
 * Description    : This event indicates that a new connection has been created
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{
  
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  /* Make the device connectable again. */
  set_connectable = TRUE;
  connection_handle =0;
  
  BSP_LED_On(BSP_LED1);//activity led   
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_terminate_connection();
#endif 
}/* end hci_disconnection_complete_event() */


/*******************************************************************************
 * Function Name  : aci_gatt_srv_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                           uint16_t Attr_Handle,
                                           uint16_t Attr_Data_Length,
                                           uint8_t Attr_Data[])
{
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
}

/*******************************************************************************
 * Function Name  : SensorUpdateTimeoutCB.
 * Description    : This function will be called on the expiry of 
 *                  a one-shot virtual timer.
 * Input          : See file vtimer.h
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SensorUpdateTimeoutCB(void *param)
{
  sensorTimer_expired = TRUE;
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  if (Next_State == 0x02) /* 0x02: Connection event slave */
  {
    OTA_Radio_Activity(Next_State_SysTime);  
  }
#endif 
}
