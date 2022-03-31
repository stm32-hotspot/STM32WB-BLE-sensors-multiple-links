/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_led_bar_shield.c
 * @author  MCD Application Team
 * @brief   Led bar shield Application
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
#include "app_led_bar_shield.h"
#include "stm32_seq.h"
#include "blinkt.h"

/* Private defines -----------------------------------------------------------*/ 
#define LED_BAR_SHIELD_UPDATE_PERIOD    (uint32_t)(0.005*1000*1000/CFG_TS_TICK_VAL) /*5ms*/

/* Private variables ---------------------------------------------------------*/   
uint8_t Led_bar_shield_Timer_Id;

uint8_t Led_bar_scene, Led_bar_scene_next;

/* Private function prototypes -----------------------------------------------*/
static void Led_bar_shield_Timer_Callback(void);
static void Led_bar_shield_update(void);

/**
  * @brief  Led bar shield Initialization.
  */
void Led_bar_shield_Init(void)
{
  BLINKT_Init(0, GPIOA, GPIO_PIN_5 , GPIOA, GPIO_PIN_2);
  BLINKT_Init(1, GPIOC, GPIO_PIN_1 , GPIOC, GPIO_PIN_4);
  BLINKT_Init(2, GPIOE, GPIO_PIN_4,  GPIOD, GPIO_PIN_14);
  BLINKT_Init(3, GPIOB, GPIO_PIN_10, GPIOE, GPIO_PIN_0);
    
  UTIL_SEQ_RegTask( 1<<CFG_TASK_LED_BAR_SHIELD_UPDATE_ID, UTIL_SEQ_RFU, Led_bar_shield_update);
  
  HW_TS_Create(CFG_TIM_PROC_ID_ISR,
        &Led_bar_shield_Timer_Id,
        hw_ts_Repeated,
        Led_bar_shield_Timer_Callback);
  
  Led_bar_scene = 1;
  Led_bar_scene_next = 0;
  
  return;
}
               
void Led_bar_shield_Start(void)
{
  /* Start the timer used to update Led bar display */
  HW_TS_Start(Led_bar_shield_Timer_Id, LED_BAR_SHIELD_UPDATE_PERIOD);
}
  
void Led_bar_shield_Stop(void)
{
  /* Stop the timer used to update Led bar display */
  HW_TS_Stop(Led_bar_shield_Timer_Id);
}

void Led_bar_set_scene(uint8_t sce)
{
  Led_bar_scene_next = sce;
  
  return;
}
        
/**
 * @brief  On timeout, trigger the task
 *         to update the led display
 * @param  None
 * @retval None
 */
static void Led_bar_shield_Timer_Callback(void)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_LED_BAR_SHIELD_UPDATE_ID, CFG_SCH_PRIO_0);
}

volatile uint32_t led_val = 0;
volatile uint32_t led_val2 = 0;
volatile uint32_t led_val3 = 0;

static void Led_bar_shield_update(void)
{
  uint32_t limit;
  
  if(Led_bar_scene != Led_bar_scene_next)
  {
    Led_bar_scene = Led_bar_scene_next;
    
    led_val = 0;
    led_val2 = 0;
    led_val3 = 0;
    
    switch(Led_bar_scene)
    {
    case LED_SCENE_RAINBOW:
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_RAINBOW, 360 / 16 / 2 );
      break;
    case LED_SCENE_ROT_PINK:
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_ROT_PINK, led_val2);
      break;
    case LED_SCENE_ROT_YELLOW:
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_ROT_YELLOW, led_val2);
      break;
    case LED_SCENE_ROT_BLUE:
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_ROT_BLUE, led_val2);
      break;
    case LED_SCENE_ROT_WHITE:
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_ROT_WHITE, led_val2);
      break;
    default:
      break;
    }
  }
  
  switch(Led_bar_scene)
  {
  case LED_SCENE_RAINBOW:
    if(led_val >= 360)
    {
      led_val = 0;
    }
    
    BLINKT_AnimationStep(led_val);
    led_val++;
    break;
  case LED_SCENE_ROT_PINK:
  case LED_SCENE_ROT_YELLOW:
  case LED_SCENE_ROT_BLUE:
  case LED_SCENE_ROT_WHITE:
    limit = ((BAR_INSTANCE * LEDS_COUNT * 4) + (led_val3 * 3));
    if((led_val / 8) >= limit )
    {
      led_val = 0;
      led_val2 = !led_val2;    
      led_val3 = (uint32_t)(READ_BIT(RTC->SSR, RTC_SSR_SS)) % (BAR_INSTANCE * LEDS_COUNT);
    }  
    if((led_val % 8) == 0)
    {
      BLINKT_AnimationStep(led_val2);
    }
    led_val++;
    break;
  default:
    break;
  }  

  return;
}
