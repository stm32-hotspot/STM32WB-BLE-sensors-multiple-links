/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motionfx_server_app.c
  * @author  MCD Application Team
  * @brief   Handle SW/Sensor Data Fusion and ECompass Service/Char
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */  

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "ble.h"
#include "dbg_trace.h"

#include "motenv_server_app.h"
#include "motionfx_server_app.h"

#include "MotionFX_Manager.h"

#include "stm32wb5mm_dk.h"
#include "stm32wb5mm_dk_lcd.h"
#include "stm32_lcd.h"
#include "stm32wb5mm_dk_motion_sensors.h"
/* Private defines -----------------------------------------------------------*/
/**
 * @brief Define The transmission interval in Multiple of 10ms for quaternions
 */
#define QUAT_UPDATE_MUL_10MS (3)
   
/**
 * @brief Define How Many quaterions you want to trasmit (from 1 to 3)
 */
#define SEND_N_QUATERNIONS (3)

#define MOTIONFX_ENGINE_DELTATIME       0.050f

/**
 * @brief Algorithm period [ms]
 */
#define MOTIONFX_ALGO_PERIOD            (50)

#define VALUE_LEN_QUAT          (2+6*SEND_N_QUATERNIONS)
#define VALUE_LEN_ECOMPASS      (2+2)

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  SW/Sensor Data Fusion Service/Char Context structure definition
 */
typedef struct
{
  uint8_t  QuatNotificationStatus;
  uint8_t  ECompassNotificationStatus;

  MOTION_SENSOR_Axes_t MAG_Offset;
  uint32_t MagTimeStamp;
  uint8_t MagCalStatus;

  MOTION_SENSOR_Axes_t quat_axes[SEND_N_QUATERNIONS];
  uint16_t Angle; /* ECompass */
  
  MOTION_SENSOR_Axes_t acceleration;
  MOTION_SENSOR_Axes_t angular_velocity;
  MOTION_SENSOR_Axes_t magnetic_field;
  uint8_t hasAcc;
  uint8_t hasGyro;
  uint8_t hasMag;
} MOTIONFX_Server_App_Context_t;

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

PLACE_IN_SECTION("BLE_APP_CONTEXT") static MOTIONFX_Server_App_Context_t MOTIONFX_Server_App_Context;
static uint8_t MOTIONFX_stopDisplayValue;

/* Global variables ----------------------------------------------------------*/
extern int debug_trace_enabled;
extern uint8_t manuf_data[14];

/* Private function prototypes -----------------------------------------------*/
//static void MagCalibTest(void);
static void ComputeQuaternions(void);
static void Quat_Update(MOTION_SENSOR_Axes_t *data);
static void MOTIONFX_Handle_Sensor(void);
static void MOTIONFX_GetCaps(void);

/* Functions Definition ------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Init the SW/Sensor Data Fusion Service/Char Context
 *         and update the ADV data accordingly
 * @param  None
 * @retval None
 */
void MOTIONFX_Context_Init(void)
{
  /* Motion Sensors */
  MOTIONFX_Server_App_Context.hasAcc = 0;
  MOTIONFX_Server_App_Context.hasGyro = 0;
  MOTIONFX_Server_App_Context.hasMag = 0;
  
  /* Check Motion caps */
  MOTIONFX_GetCaps();
  
  MOTIONFX_Server_App_Context.MagTimeStamp = 0;
  MOTIONFX_Server_App_Context.MagCalStatus = 0;

  /* Sensor Fusion API initialization function */
  MotionFX_manager_init();
  MotionFX_manager_start_6X();

  /* Enable magnetometer calibration */
  //MagCalibTest();

  /* Update BLE ADV field (MotionFx) */
  manuf_data[6] |= 0x01; /* Sensor fusion*/
  //manuf_data[7] |= 0x40; /* ECompass*/

  MOTIONFX_Set_Quat_Notification_Status(0);
  MOTIONFX_Set_ECompass_Notification_Status(0);
}

/**
 * @brief  Set the notification status (enabled/disabled) for Sensor Data Fusion
 * @param  status The new notification status
 * @retval None
 */
void MOTIONFX_Set_Quat_Notification_Status(uint8_t status)
{
  MOTIONFX_Server_App_Context.QuatNotificationStatus = status;
  if(status == 0){
    MOTIONFX_stopDisplayValue = 1;
  }else{
    MOTIONFX_stopDisplayValue = 0;
  }
}

/**
 * @brief  Set the notification status (enabled/disabled) for ECompass
 * @param  status The new notification status
 * @retval None
 */
void MOTIONFX_Set_ECompass_Notification_Status(uint8_t status)
{
  MOTIONFX_Server_App_Context.ECompassNotificationStatus = status;
}

/**
 * @brief  Send a notification for Quaternions (Sensor Data Fusion case)
 * @param  None
 * @retval None
 */
void MOTIONFX_Send_Quat_Notification_Task(void)
{
  ComputeQuaternions();
}

/**
 * @brief  Send a notification for Quaternions (ECompass case)
 * @param  None
 * @retval None
 */
void MOTIONFX_Send_ECompass_Notification_Task(void)
{
  ComputeQuaternions();
}

/**
 * @brief  Return the Magneto Calibration status
 * @param  None
 * @retval Magneto Calibration status
 */
uint8_t MOTIONFX_Get_MagCalStatus(void)
{
  return MOTIONFX_Server_App_Context.MagCalStatus;
}

/**
 * @brief  Return the Magneto Calibration offset
 * @param  None
 * @retval Magneto Calibration offset
 */
MOTION_SENSOR_Axes_t *MOTIONFX_Get_MAG_Offset(void)
{
  return &(MOTIONFX_Server_App_Context.MAG_Offset);
}

/**
 * @brief  Force Magneto Calibration
 * @param  None
 * @retval None
 */
void MOTIONFX_ReCalibration(void)
{
  /* Reset Magneto Calibration */
  MOTIONFX_Server_App_Context.MagCalStatus = 0;
/*
  CONFIG_Send_Notification(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, 0);
  CONFIG_Send_Notification(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, 0);
*/
  /* Enable Magneto calibration */
  MotionFX_manager_MagCal_start(MOTIONFX_ALGO_PERIOD);
}

/* Private functions ---------------------------------------------------------*/

/** 
 * @brief  MotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;

  static int32_t CounterFX = 0;
  static int32_t CounterEC = 0;
  
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;

   /* Increment the Counter */
  if(MOTIONFX_Server_App_Context.QuatNotificationStatus)
  {
    CounterFX++;
  }
  else if(MOTIONFX_Server_App_Context.ECompassNotificationStatus)
  {
    CounterEC++;
  }
  
  MOTIONFX_Handle_Sensor();
  
  if(MOTIONFX_Server_App_Context.hasAcc == 1)
  {
    ACC_Value.x = -MOTIONFX_Server_App_Context.acceleration.y;
    ACC_Value.y = MOTIONFX_Server_App_Context.acceleration.x;
    ACC_Value.z = MOTIONFX_Server_App_Context.acceleration.z;
    
    data_in.acc[0] = (float)ACC_Value.x * FROM_MG_TO_G;
    data_in.acc[1] = (float)ACC_Value.y * FROM_MG_TO_G;
    data_in.acc[2] = (float)ACC_Value.z * FROM_MG_TO_G;
  }

  if(MOTIONFX_Server_App_Context.hasGyro == 1)
  {
    MOTIONFX_Server_App_Context.angular_velocity.x /= 100;
    MOTIONFX_Server_App_Context.angular_velocity.y /= 100;
    MOTIONFX_Server_App_Context.angular_velocity.z /= 100;

    GYR_Value.x = MOTIONFX_Server_App_Context.angular_velocity.y;
    GYR_Value.y = MOTIONFX_Server_App_Context.angular_velocity.x;
    GYR_Value.z = MOTIONFX_Server_App_Context.angular_velocity.z;
    
    data_in.gyro[0] = (float)GYR_Value.x * FROM_MDPS_TO_DPS;
    data_in.gyro[1] = (float)GYR_Value.y * FROM_MDPS_TO_DPS;
    data_in.gyro[2] = (float)GYR_Value.z * FROM_MDPS_TO_DPS;
  }
  
  if(MOTIONFX_Server_App_Context.hasMag == 1)
  {
    MAG_Value.x = MOTIONFX_Server_App_Context.magnetic_field.x;
    MAG_Value.y = MOTIONFX_Server_App_Context.magnetic_field.y;
    MAG_Value.z = MOTIONFX_Server_App_Context.magnetic_field.z;

    data_in.mag[0] = (float)MAG_Value.x * FROM_MGAUSS_TO_UT50;
    data_in.mag[1] = (float)MAG_Value.y * FROM_MGAUSS_TO_UT50;
    data_in.mag[2] = (float)MAG_Value.z * FROM_MGAUSS_TO_UT50;
  }
  
  /* Run Sensor Fusion algorithm */
  MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);
  
  if(MOTIONFX_Server_App_Context.QuatNotificationStatus)
  {
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if(pdata_out->quaternion_6X[3] < 0)
    {
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion_6X[0] * (-10000));
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion_6X[1] * (-10000));
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion_6X[2] * (-10000));
    }
    else
    {
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion_6X[0] * 10000);
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion_6X[1] * 10000);
      MOTIONFX_Server_App_Context.quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion_6X[2] * 10000);
    }

    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions informations via bluetooth */
    if(CounterFX == QUAT_UPDATE_MUL_10MS)
    {
      Quat_Update(MOTIONFX_Server_App_Context.quat_axes);
      CounterFX = 0;
    }
  }
}

/**
 * @brief  Update quaternions characteristic value
 * @param  data Structure containing the quaterions
 * @retval None
 */
static void Quat_Update(MOTION_SENSOR_Axes_t *data)
{
  uint8_t value[VALUE_LEN_QUAT];

  /* Timestamp */
  STORE_LE_16(value, (HAL_GetTick()>>3));
  
  STORE_LE_16(value+2,data[0].x);
  STORE_LE_16(value+4,data[0].y);
  STORE_LE_16(value+6,data[0].z);

  STORE_LE_16(value+8 ,data[1].x);
  STORE_LE_16(value+10,data[1].y);
  STORE_LE_16(value+12,data[1].z);

  STORE_LE_16(value+14,data[2].x);
  STORE_LE_16(value+16,data[2].y);
  STORE_LE_16(value+18,data[2].z);

  if(MOTIONFX_Server_App_Context.QuatNotificationStatus)
  {
    if(debug_trace_enabled == 1){
      APP_DBG_MSG("-- MOTIONFX APPLICATION SERVER : NOTIFY CLIENT WITH NEW QUAT PARAMETER VALUE \n ");
      APP_DBG_MSG(" \n\r");
    }
    MOTENV_STM_App_Update_Char(MOTION_FX_CHAR_UUID, VALUE_LEN_QUAT, (uint8_t *)&value);
  }
  else
  {
    if(debug_trace_enabled == 1){
      APP_DBG_MSG("-- MOTIONFX APPLICATION SERVER : CAN'T INFORM CLIENT - NOTIFICATION DISABLED\n ");
    }
  }

  return;
}

///**
// * @brief  Test if calibration data are available
// * @param  None
// * @retval None
// */
//static void MagCalibTest(void)
//{
//  MFX_MagCal_output_t mag_cal_test;
//  
//  /* Recall the calibration Credential saved */
//  MotionFX_manager_MagCal_start(MOTIONFX_ALGO_PERIOD);
//  MotionFX_MagCal_getParams(&mag_cal_test);
//    
//  if(mag_cal_test.cal_quality == MFX_MAGCALGOOD)
//  {
//    MOTIONFX_Server_App_Context.MAG_Offset.x = (int32_t) (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
//    MOTIONFX_Server_App_Context.MAG_Offset.y = (int32_t) (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
//    MOTIONFX_Server_App_Context.MAG_Offset.z = (int32_t) (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
//    
//    MOTIONFX_Server_App_Context.MagCalStatus = 1;
//    
//#if(CFG_DEBUG_APP_TRACE != 0)
//    APP_DBG_MSG("-- MOTIONFX APPLICATION SERVER : Magneto Calibration Read\r\n");
//#endif
//  }
//  else
//  {
//    MOTIONFX_Server_App_Context.MagCalStatus = 0;
//#if(CFG_DEBUG_APP_TRACE != 0)
//    APP_DBG_MSG("-- MOTIONFX APPLICATION SERVER : Magneto Calibration quality is not good\r\n");
//#endif
//  }
//  
//  if(!MOTIONFX_Server_App_Context.MagCalStatus)
//  {
//    MOTIONFX_Server_App_Context.MAG_Offset.x = 0;
//    MOTIONFX_Server_App_Context.MAG_Offset.y = 0;
//    MOTIONFX_Server_App_Context.MAG_Offset.z = 0;
//  }
//}

/**
 * @brief  Parse the values read by Motion sensors
 * @param  None
 * @retval None
 */
static void MOTIONFX_Handle_Sensor(void)
{
  MOTION_SENSOR_AxesRaw_t accelerationRaw;
  MOTION_SENSOR_AxesRaw_t angular_velocityRaw;
  MOTION_SENSOR_Axes_t magnetic_field;
  
  char accValue[18];
  char gyroValue[18];
  float accX,accY,accZ,gyroX,gyroY,gyroZ;
  
  if(MOTIONFX_Server_App_Context.hasAcc == 1)
  {
    memset(&accelerationRaw, 0, sizeof(MOTION_SENSOR_AxesRaw_t));
    BSP_MOTION_SENSOR_GetAxesRaw(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, &accelerationRaw);

    MOTIONFX_Server_App_Context.acceleration.x = (int32_t)((float)accelerationRaw.x * ISM330DHCX_ACC_SENSITIVITY_FS_2G);
    MOTIONFX_Server_App_Context.acceleration.y = (int32_t)((float)accelerationRaw.y * ISM330DHCX_ACC_SENSITIVITY_FS_2G);
    MOTIONFX_Server_App_Context.acceleration.z = (int32_t)((float)accelerationRaw.z * ISM330DHCX_ACC_SENSITIVITY_FS_2G);
  }
  
  if(MOTIONFX_Server_App_Context.hasGyro == 1)
  {
    memset(&angular_velocityRaw, 0, sizeof(MOTION_SENSOR_AxesRaw_t));
    BSP_MOTION_SENSOR_GetAxesRaw(MOTION_SENSOR_ISM330DHCX_0, MOTION_GYRO, &angular_velocityRaw);
    
    MOTIONFX_Server_App_Context.angular_velocity.x = (int32_t)((float)angular_velocityRaw.x * ISM330DHCX_GYRO_SENSITIVITY_FS_2000DPS);
    MOTIONFX_Server_App_Context.angular_velocity.y = (int32_t)((float)angular_velocityRaw.y * ISM330DHCX_GYRO_SENSITIVITY_FS_2000DPS);
    MOTIONFX_Server_App_Context.angular_velocity.z = (int32_t)((float)angular_velocityRaw.z * ISM330DHCX_GYRO_SENSITIVITY_FS_2000DPS);
  }

  if(MOTIONFX_Server_App_Context.hasMag == 1)
  {
    magnetic_field.x = 1;
    magnetic_field.y = 1;
    magnetic_field.z = 1;
    MOTIONFX_Server_App_Context.magnetic_field = magnetic_field;
  }

  MOTIONFX_stopDisplayValue = 1;
  if(MOTIONFX_stopDisplayValue == 0)
  {    
    if(MOTIONFX_Server_App_Context.hasAcc == 1)
    {
      accX = (float)MOTIONFX_Server_App_Context.acceleration.x;
      accY = (float)MOTIONFX_Server_App_Context.acceleration.y;
      accZ = (float)MOTIONFX_Server_App_Context.acceleration.z;
      sprintf(accValue,"%5.0f|%5.0f|%5.0f",accX,accY,accZ);
    }
    
    if(MOTIONFX_Server_App_Context.hasGyro == 1)
    {
      gyroX = (float)((MOTIONFX_Server_App_Context.angular_velocity.x) / 100.0);
      gyroY = (float)((MOTIONFX_Server_App_Context.angular_velocity.y) / 100.0);
      gyroZ = (float)((MOTIONFX_Server_App_Context.angular_velocity.z) / 100.0);
      sprintf(gyroValue,"%5.0f|%5.0f|%5.0f",gyroX,gyroY,gyroZ);
    }
    
    UTIL_LCD_ClearStringLine(2);
    UTIL_LCD_ClearStringLine(3);
    UTIL_LCD_ClearStringLine(4);
       
    UTIL_LCD_DisplayStringAtLine(2,(uint8_t*)"Accelero and Gyro");
    UTIL_LCD_DisplayStringAtLine(3,(uint8_t*)accValue);
    UTIL_LCD_DisplayStringAtLine(4,(uint8_t*)gyroValue);
      
    BSP_LCD_Refresh(0);
  }
}

/**
 * @brief  Check the Motion active capabilities and set the ADV data accordingly
 * @param  None
 * @retval None
 */
static void MOTIONFX_GetCaps(void)
{
  MOTIONFX_Server_App_Context.hasMag = 0;
  MOTIONFX_Server_App_Context.hasGyro = 1;
  MOTIONFX_Server_App_Context.hasAcc = 1;
}
