/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
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
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "p2p_server_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motenv_server_app.h"
#include "app_vl53l0x.h"
#include "app_led_bar_shield.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define CONN_UPDATE_TIMEOUT            (5*1000*1000/CFG_TS_TICK_VAL) /**< 5s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

PLACE_IN_SECTION("BLE_APP_CONTEXT") BleApplicationContext_t BleApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;

P2PS_APP_ConnHandle_Not_evt_t handleNotification;

#if L2CAP_REQUEST_NEW_CONN_PARAM != 0
#define SIZE_TAB_CONN_INT            2
float tab_conn_interval[SIZE_TAB_CONN_INT] = {50, 1000} ; /* ms */
uint8_t index_con_int, mutex;
#endif

/**
 * Advertising Data
 */
#if (P2P_SERVER1 != 0)
static const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME ,'W','B','5','M',' ','C','U', 'B','E'};
uint8_t manuf_data[14] = {
    sizeof(manuf_data)-1, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0x01/*SKD version */,
    CFG_DEV_ID_P2P_SERVER1 /* STM32WB - P2P Server 1*/,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
};
#endif

/* USER CODE BEGIN PV */
static volatile uint8_t link_connected = 0;
static volatile uint8_t adv_active = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx( void * pPayload );
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress( void );
static void Adv_Request( APP_BLE_ConnStatus_t New_Status );
static void Adv_Cancel( void );
static void Adv_Cancel_Req( void );
static void Switch_OFF_GPIO( void );
static void Conn_Interval_Update(uint16_t Connection_Handle, uint16_t interval_min, uint16_t interval_max);
static void Conn_PHY_2M_Update(uint16_t Connection_Handle);

static void Conn_Update_Link0(void);
static void Conn_Update_Link1(void);
static void Conn_Update_Link2(void);
static void Conn_Update_Link3(void);
static void Conn_Update_Link4(void);
static void Conn_Update_Link5(void);
static void Conn_Update_Link6(void);
static void Conn_Update_Link7(void);

/* USER CODE BEGIN PFP */
static void diplay_link_connected( void );
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init( void )
{
/* USER CODE BEGIN APP_BLE_Init_1 */
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;  
  char BdAddress[20];
  char StackVersion[23];
  char StackBranch[20];
  char FusVersion[20];
  const uint8_t *bdaddr;  
  uint8_t i;
/* USER CODE END APP_BLE_Init_1 */
  memset(&bd_addr_udn[0], 0xFF, BD_ADDR_SIZE_LOCAL);
  
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_SLAVE_SCA,
     CFG_BLE_MASTER_SCA,
     CFG_BLE_LSE_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG}
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  if (SHCI_C2_BLE_Init( &ble_init_cmd_packet ) != SHCI_Success)
  {
    Error_Handler();
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

  for ( i = 0 ; i < CONNECTION_COUNT ; i++)
  {
    BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle = 0xFFFF;
    BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionInterval = 0;
    BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionPhy = 0;
  }
  
  /**
   * From here, all initialization are BLE application specific
   */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);
  
  /* creation of a timer for each link, used for the peripheral to ask for a connection update */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[0].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link0);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[1].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link1);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[2].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link2);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[3].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link3);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[4].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link4);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[5].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link5);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[6].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link6);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[7].Update_conn_timer_Id), hw_ts_SingleShot, Conn_Update_Link7);
 
  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
  manuf_data[sizeof(manuf_data)-8] = CFG_FEATURE_OTA_REBOOT;
#endif
#if(RADIO_ACTIVITY_EVENT != 0)
  aci_hal_set_radio_activity_mask(0x0006);
#endif

  /**
   * Initialize P2P Server Application
   */
  P2PS_APP_Init();

/* USER CODE BEGIN APP_BLE_Init_3 */
  /**
  * Initialize MOTENV Server Application
  */
  MOTENV_APP_Init();
/* USER CODE END APP_BLE_Init_3 */

  /**
   * Create timer to handle the Advertising Stop
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Cancel_Req);
  /**
   * Create timer to handle the Led Switch OFF
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.SwitchOffGPIO_timer_Id), hw_ts_SingleShot, Switch_OFF_GPIO);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by P2P Client
   */
   Adv_Request(APP_BLE_FAST_ADV);

/* USER CODE BEGIN APP_BLE_Init_2 */
   UTIL_SEQ_RegTask( 1<<CFG_TASK_DISPLAY_LINK_ID, UTIL_SEQ_RFU, diplay_link_connected);
   UTIL_SEQ_SetTask(1<<CFG_TASK_DISPLAY_LINK_ID, CFG_SCH_PRIO_0);
   
   /* Displays the board information: MAC Address, Stack version, FUS version*/ 
   if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
   {
     // Error
   }
   else
   {
     bdaddr= BleGetBdAddress();
     sprintf(BdAddress, "BD_ad=%02x%02x%02x%02x%02x%02x", bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
     sprintf(StackVersion, "BLE Stack=v%d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);
     sprintf(StackBranch, "Branch=%d Type=%d", p_wireless_info->VersionBranch, p_wireless_info->VersionReleaseType);
     sprintf(FusVersion, "FUS v%d.%d.%d", p_wireless_info->FusVersionMajor, p_wireless_info->FusVersionMinor, p_wireless_info->FusVersionSub);
     
     BSP_LCD_Clear(0,SSD1315_COLOR_BLACK);
     BSP_LCD_Refresh(0);
     UTIL_LCD_DisplayStringAt(0, 0, (uint8_t *)BdAddress, LEFT_MODE);
     UTIL_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)StackVersion, LEFT_MODE);
     UTIL_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)StackBranch, LEFT_MODE);
     UTIL_LCD_DisplayStringAt(0, LINE(3), (uint8_t *)FusVersion, LEFT_MODE);
     BSP_LCD_Refresh(0);
   }
   HAL_Delay(4000);
   /* Displays Application */
   BSP_LCD_Clear(0,SSD1315_COLOR_BLACK);
   BSP_LCD_Refresh(0);
   UTIL_LCD_DisplayStringAt(0, 0, (uint8_t *)"WB BLE Sensor", CENTER_MODE);
   BSP_LCD_Refresh(0);
   
   VL53L0X_Start_Measure();
#if (CFG_LED_BAR_SHIELD_SUPPORTED == 1)
   Led_bar_shield_Start();
#endif
/* USER CODE END APP_BLE_Init_2 */
  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  evt_blecore_aci *blecore_evt;
  uint8_t i;

  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  switch (event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      hci_disconnection_complete_event_rp0 *disconnection_complete_event;
      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) event_pckt->data;

      for ( i = 0 ; i < CONNECTION_COUNT ; i++)
      {
        if (disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle)
        {
          APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH CLIENT conn handle: %x\n", disconnection_complete_event->Connection_Handle);
          BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle = 0xFFFF;
          handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle;
          BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;          
          break;
        }
      }

      /* restart advertising */
      if(adv_active != 0)
      {
        Adv_Request(APP_BLE_FAST_ADV);
      }

      /**
       * SPECIFIC to P2P Server APP
       */
      handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
      
      P2PS_APP_Notification(&handleNotification);
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */
      link_connected--;
      
      if(link_connected == 0)
      {
        VL53L0X_Start_Measure();
      }
      
      UTIL_SEQ_SetTask(1<<CFG_TASK_DISPLAY_LINK_ID, CFG_SCH_PRIO_0);
      /* USER CODE END EVT_DISCONN_COMPLETE */
    }

    break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
    {
      meta_evt = (evt_le_meta_event*) event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("\r\n**EVENT CONNECTION UPDATE complete\r\n");

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */
          /* Check connection update parameters */
          hci_le_connection_update_complete_event_rp0 *connection_update_complete_event;
          connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) meta_evt->data;
          
          APP_DBG_MSG("  conn handle: 0x%04x", connection_update_complete_event->Connection_Handle);
          if (connection_update_complete_event->Status == 0)
          {
            APP_DBG_MSG("  connection interval: %d ms\r\n", (uint8_t)(connection_update_complete_event->Conn_Interval*1.25));
          }
          else
          {
            APP_DBG_MSG("  status KO\r\n");
          }
          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;
        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
          {
          hci_le_phy_update_complete_event_rp0 *evt_le_phy_update_complete;
  
          APP_DBG_MSG("\r\n**EVENT PHY  UPDATE complete\r\n");
          evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)meta_evt->data;
          
          APP_DBG_MSG("  conn handle: 0x%04x", evt_le_phy_update_complete->Connection_Handle);
          if (evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG(" RX_PHY: %d", evt_le_phy_update_complete->RX_PHY);
            APP_DBG_MSG(" TX_PHY: %d\r\n", evt_le_phy_update_complete->TX_PHY);
          }
          else
          {
            APP_DBG_MSG("  status KO\r\n");
          }


          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;
          }
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_complete_event_rp0 *connection_complete_event;

          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;

          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          for ( i = 0 ; i < CONNECTION_COUNT ; i++)
          {
            if (BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle == 0xFFFF)
            {
              BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle = connection_complete_event->Connection_Handle;
              handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].connectionHandle;
              HW_TS_Start(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[i].Update_conn_timer_Id, CONN_UPDATE_TIMEOUT);
              break;
            }            
          }
          APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n", connection_complete_event->Connection_Handle);
          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          /**
           * SPECIFIC to P2P Server APP
           */
          handleNotification.P2P_Evt_Opcode = PEER_CONN_HANDLE_EVT;
          P2PS_APP_Notification(&handleNotification);
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */
          link_connected++;
                    
          if(link_connected == 1)
          {
            VL53L0X_Stop_Measure();
          }
          
          if(link_connected < 8)
          {
            if(adv_active != 0)
            {
              Adv_Request(APP_BLE_FAST_ADV);
            }
          }
          
          UTIL_SEQ_SetTask(1<<CFG_TASK_DISPLAY_LINK_ID, CFG_SCH_PRIO_0);
          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
        }
        break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

        /* USER CODE BEGIN META_EVT */

        /* USER CODE END META_EVT */

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }
    }
    break; /* HCI_LE_META_EVT_CODE */

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blecore_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */
      /**
       * SPECIFIC to P2P Server APP
       */
        case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
        /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */

        /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */
        break;
        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
        APP_DBG_MSG("\r\n\r** ACI_GAP_PROC_COMPLETE_VSEVT_CODE \n");
        /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

        /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */
#if(RADIO_ACTIVITY_EVENT != 0)
        case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
        /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/

        /* USER CODE END RADIO_ACTIVITY_EVENT*/
          break; /* ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE */
#endif

      /* USER CODE BEGIN BLUE_EVT */

      /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

      default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/
void APP_BLE_Key_Button1_Action(void)
{
  P2PS_APP_SW1_Button_Action();
}

uint8_t App_Led_bar_scene, App_Led_bar_scene_next;

void APP_BLE_Key_Button2_Action(void)
{
  if(adv_active != 0)
  {
    aci_gap_set_non_discoverable();
    adv_active = 0;
  }
  else
  {
    Adv_Request(APP_BLE_FAST_ADV);
    adv_active = 1;
  }  
  
  App_Led_bar_scene_next = (App_Led_bar_scene + 1) % LED_SCENE_MAX;
  Led_bar_set_scene(App_Led_bar_scene_next);
  App_Led_bar_scene = App_Led_bar_scene_next;
  
}
/* USER CODE END FD*/
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE };

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);

#if (CFG_BLE_ADDRESS_TYPE == PUBLIC_ADDR)
  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = bd_addr[5];
  manuf_data[ sizeof(manuf_data)-5] = bd_addr[4];
  manuf_data[ sizeof(manuf_data)-4] = bd_addr[3];
  manuf_data[ sizeof(manuf_data)-3] = bd_addr[2];
  manuf_data[ sizeof(manuf_data)-2] = bd_addr[1];
  manuf_data[ sizeof(manuf_data)-1] = bd_addr[0];
#endif

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#elif (CFG_BLE_ADDRESS_TYPE == RANDOM_ADDR)
  /* Get RNG semaphore */
  while( LL_HSEM_1StepLock( HSEM, CFG_HW_RNG_SEMID ) );

  /* Enable RNG */
  __HAL_RNG_ENABLE(&hrng);

  /* Enable HSI48 oscillator */
  LL_RCC_HSI48_Enable();
  /* Wait until HSI48 is ready */
  while( ! LL_RCC_HSI48_IsReady( ) );

  if (HAL_RNG_GenerateRandomNumber(&hrng, &srd_bd_addr[1]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  if (HAL_RNG_GenerateRandomNumber(&hrng, &srd_bd_addr[0]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */

  /* Disable HSI48 oscillator */
  LL_RCC_HSI48_Disable();

  /* Disable RNG */
  __HAL_RNG_DISABLE(&hrng);

  /* Release RNG semaphore */
  LL_HSEM_ReleaseLock( HSEM, CFG_HW_RNG_SEMID, 0 );
#endif

#if (CFG_BLE_ADDRESS_TYPE == STATIC_RANDOM_ADDR)
  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = srd_bd_addr[1] >> 8 ;
  manuf_data[ sizeof(manuf_data)-5] = srd_bd_addr[1];
  manuf_data[ sizeof(manuf_data)-4] = srd_bd_addr[0] >> 24;
  manuf_data[ sizeof(manuf_data)-3] = srd_bd_addr[0] >> 16;
  manuf_data[ sizeof(manuf_data)-2] = srd_bd_addr[0] >> 8;
  manuf_data[ sizeof(manuf_data)-1] = srd_bd_addr[0];

  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );
#endif

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
  aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {
    const char *name = "WB5M DK";
    aci_gap_init(role,
#if ((CFG_BLE_ADDRESS_TYPE == RESOLVABLE_PRIVATE_ADDR) || (CFG_BLE_ADDRESS_TYPE == NON_RESOLVABLE_PRIVATE_ADDR))
                 2,
#else
                 0,
#endif
                 APPBLE_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle,
                 &gap_dev_name_char_handle,
                 &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }
  /**
   * Initialize Default PHY
   */
  hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                         CFG_SC_SUPPORT,
                                         CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         CFG_BLE_ADDRESS_TYPE
                                         );

  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }
}

static void Adv_Request(APP_BLE_ConnStatus_t New_Status)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  if (New_Status == APP_BLE_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

    /**
     * Stop the timer, it will be restarted for a new shot
     * It does not hurt if the timer was not running
     */
    HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

    APP_DBG_MSG("First index in %d state \n", BleApplicationContext.Device_Connection_Status);

    if ((New_Status == APP_BLE_LP_ADV)
        && ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
            || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
    {
      /* Connection in ADVERTISE mode have to stop the current advertising */
      ret = aci_gap_set_non_discoverable();
      if (ret == BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("Successfully Stopped Advertising \n");
      }
      else
      {
        APP_DBG_MSG("Stop Advertising Failed , result: %d \n", ret);
      }
    }

    BleApplicationContext.Device_Connection_Status = New_Status;
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_discoverable(
        ADV_IND,
        Min_Inter,
        Max_Inter,
        CFG_BLE_ADDRESS_TYPE,
        NO_WHITE_LIST_USE, /* use white list */
        sizeof(local_name),
        (uint8_t*) &local_name,
        BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
        BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
        0,
        0);

    /* Update Advertising data */
    ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);
    if (ret == BLE_STATUS_SUCCESS)
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
        APP_DBG_MSG("Successfully Start Fast Advertising \n" );
        /* Start Timer to STOP ADV - TIMEOUT */
        //HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
      }
      else
      {
        APP_DBG_MSG("Successfully Start Low Power Advertising \n");
      }
    }
    else
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
        APP_DBG_MSG("Start Fast Advertising Failed , result: %d \n", ret);
      }
      else
      {
        APP_DBG_MSG("Start Low Power Advertising Failed , result: %d \n", ret);
      }
    }

  return;
}

const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;
  
  if(bd_addr_udn[5] != 0xFF)
  {
    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    udn = LL_FLASH_GetUDN();
    
    if( (HAL_GPIO_ReadPin(BUTTON_USER2_GPIO_PORT, BUTTON_USER2_PIN) == GPIO_PIN_RESET) &&
       (bd_addr_udn[5] == 0xFF))
    {
      udn = ((uint32_t)(READ_BIT(RTC->SSR, RTC_SSR_SS)));
    }

    if(udn != 0xFFFFFFFF)
    {
      company_id = LL_FLASH_GetSTCompanyID();
      device_id = LL_FLASH_GetDeviceID();

  /**
   * Public Address with the ST company ID
   * bit[47:24] : 24bits (OUI) equal to the company ID
   * bit[23:16] : Device ID.
   * bit[15:0] : The last 16bits from the UDN
   * Note: In order to use the Public Address in a final product, a dedicated
   * 24bits company ID (OUI) shall be bought.
   */
      bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
      bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
      bd_addr_udn[2] = (uint8_t)device_id;
      bd_addr_udn[3] = (uint8_t)(company_id & 0x000000FF);
      bd_addr_udn[4] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );
      bd_addr_udn[5] = (uint8_t)( (company_id & 0x00FF0000) >> 16 );

      bd_addr = (const uint8_t *)bd_addr_udn;
    }
    else
    {
      otp_addr = OTP_Read(0);
      if(otp_addr)
      {
        bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
      }
      else
      {
        bd_addr = M_bd_addr;
      }
    }
  }

  return bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 *SPECIFIC FUNCTIONS FOR P2P SERVER
 *
 *************************************************************/
static void Adv_Cancel( void )
{
/* USER CODE BEGIN Adv_Cancel_1 */

/* USER CODE END Adv_Cancel_1 */

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_SERVER)

  {

    tBleStatus result = 0x00;

    result = aci_gap_set_non_discoverable();

    BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
    if (result == BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  \r\n\r");APP_DBG_MSG("** STOP ADVERTISING **  \r\n\r");
    }
    else
    {
      APP_DBG_MSG("** STOP ADVERTISING **  Failed \r\n\r");
    }

  }

/* USER CODE BEGIN Adv_Cancel_2 */

/* USER CODE END Adv_Cancel_2 */
  return;
}

static void Adv_Cancel_Req( void )
{
/* USER CODE BEGIN Adv_Cancel_Req_1 */

/* USER CODE END Adv_Cancel_Req_1 */
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_CANCEL_ID, CFG_SCH_PRIO_0);
/* USER CODE BEGIN Adv_Cancel_Req_2 */

/* USER CODE END Adv_Cancel_Req_2 */
  return;
}

static void Switch_OFF_GPIO(){
/* USER CODE BEGIN Switch_OFF_GPIO */

/* USER CODE END Switch_OFF_GPIO */
}

static void Conn_Interval_Update(uint16_t Connection_Handle, uint16_t interval_min, uint16_t interval_max)
{
    uint16_t slave_latency = L2CAP_SLAVE_LATENCY;
    uint16_t timeout_multiplier = L2CAP_TIMEOUT_MULTIPLIER;
    tBleStatus result;

    result = aci_l2cap_connection_parameter_update_req(Connection_Handle,
                                                       CONN_P(interval_min), CONN_P(interval_max),
                                                       slave_latency, timeout_multiplier);
    
    APP_DBG_MSG("\r\n**Connection interval change request\r\n");    
    APP_DBG_MSG("  conn handle: 0x%04x  aci_l2cap_connection_parameter_update_req() to  [%d | %d] ms", Connection_Handle, (uint8_t)(interval_min), (uint8_t)(interval_max));
    if( result == BLE_STATUS_SUCCESS )
    {
      APP_DBG_MSG(" Successfully\r\n");
    }
    else
    {
      APP_DBG_MSG("Failed\r\n");
    }

  return;
}

static void Conn_Update_Link0(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[0].connectionHandle,
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[0].connectionHandle);
  return;
}

static void Conn_Update_Link1(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[1].connectionHandle, 
                               interval_min, interval_max);
  
  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[1].connectionHandle);
  return;
}

static void Conn_Update_Link2(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[2].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[2].connectionHandle);
  return;
}

static void Conn_Update_Link3(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[3].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[3].connectionHandle);
  return;
}

static void Conn_Update_Link4(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[4].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[4].connectionHandle);
  return;
}

static void Conn_Update_Link5(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[5].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[5].connectionHandle);
  return;
}

static void Conn_Update_Link6(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[6].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[6].connectionHandle);
  return;
}

static void Conn_Update_Link7(void)
{
  uint16_t interval_min = 60;
  uint16_t interval_max = 70;
  Conn_Interval_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[7].connectionHandle, 
                               interval_min, interval_max);

  Conn_PHY_2M_Update(BleApplicationContext.BleApplicationContext_legacy.ConnUpdateContext[7].connectionHandle);
  return;
}

static void Conn_PHY_2M_Update(uint16_t Connection_Handle)
{
  tBleStatus status;
  uint8_t TX_PHY, RX_PHY;
  
  APP_DBG_MSG("\r\n**PHY management\r\n");
  
  APP_DBG_MSG("  conn handle: 0x%04x hci_le_read_phy()\r\n", Connection_Handle);
  status = hci_le_read_phy(Connection_Handle,&TX_PHY,&RX_PHY);
  if (status == BLE_STATUS_SUCCESS)
  {
 
    if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
    {
      APP_DBG_MSG("  TX and RX PHY are already at 2M\r\n");
    }
    else
    {
      APP_DBG_MSG("  TX and RX PHY are NOT at 2M\r\n");
      APP_DBG_MSG("  conn handle: 0x%04x hci_le_set_phy() 2M, ", Connection_Handle);
      
      status = hci_le_set_phy(Connection_Handle, CFG_ALL_PHYS, CFG_TX_PHY, CFG_RX_PHY, 0);
      if (status == BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("Successfully\r\n");
      }
      else
      {
        APP_DBG_MSG("Failed 0x%x\r\n", status);
      }
    }
  }
  else
  {
    APP_DBG_MSG("Failed to read PHY conf\r\n");
  }
  
  return;
}
            
/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */

static void diplay_link_connected( void )
{
  char tmpText[32];
  UTIL_LCD_ClearStringLine(1);
  sprintf(tmpText,"Link connected : %d",link_connected);
  UTIL_LCD_DisplayStringAtLine(1,(uint8_t*)tmpText);
  BSP_LCD_Refresh(0);
  
  return;
}

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;
  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);

      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
