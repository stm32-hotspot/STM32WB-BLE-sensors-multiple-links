/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    blinkt.h
  * @author  MCD Application Team
  * @brief   Header for Led bar blinkt driver
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
**/
/* USER CODE END Header */

#ifndef BLINKT_H
#define BLINKT_H 100

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 * The BLINKT is a linear set of APA102 LEDs
 * https://shop.pimoroni.com/products/blinkt
 */

#include "stm32wbxx_hal.h"

#define BAR_INSTANCE        4
#define LEDS_COUNT          8
#define BAR_GRAPH_LEVEL_MAX 256

/**
 * @brief  BLINKT color enumeration
 */
typedef enum {
  BLINKT_COLOR_OFF = 0x00, 
  BLINKT_COLOR_WHITE,
  BLINKT_COLOR_RED,
  BLINKT_COLOR_GREEN,
  BLINKT_COLOR_BLUE,
  BLINKT_COLOR_MAX
} BLINKT_COLOR_t;

/**
 * @brief  BLINKT bar graph mode enumeration
 */
typedef enum {
  BLINKT_BAR_GRAPH_MODE_RED = 0,
  BLINKT_BAR_GRAPH_MODE_GREEN,
  BLINKT_BAR_GRAPH_MODE_BLUE,
  BLINKT_BAR_GRAPH_MODE_COLORS,
  BLINKT_BAR_GRAPH_MODE_MAX
} BLINKT_BAR_GRAPH_MODE_t;

/**
 * @brief  BLINKT animation mode enumeration
 */
typedef enum {
  BLINKT_ANINATION_MODE_RAINBOW = 0,
  BLINKT_ANINATION_MODE_ROT_PINK,
  BLINKT_ANINATION_MODE_ROT_YELLOW,
  BLINKT_ANINATION_MODE_ROT_BLUE,
  BLINKT_ANINATION_MODE_ROT_WHITE,
  BLINKT_ANINATION_MODE_PULSE_WHITE,
  BLINKT_ANIMATION_MODE_MAX
} BLINKT_ANIMATION_MODE_t;


enum {
  BLINKT_LED_NONE = 0,
  BLINKT_LED_0 = 0x0001 << 0,
  BLINKT_LED_1 = 0x0001 << 1,
  BLINKT_LED_2 = 0x0001 << 2,
  BLINKT_LED_3 = 0x0001 << 3,
  BLINKT_LED_4 = 0x0001 << 4,
  BLINKT_LED_5 = 0x0001 << 5,
  BLINKT_LED_6 = 0x0001 << 6,
  BLINKT_LED_7 = 0x0001 << 7,
  BLINKT_LED_ALL = 0x00FF,
  BLINKT_NO_REFRESH = 0x0001 << 8,
};

#ifndef BLINKT_COLOR_DEFAULT
#define BLINKT_COLOR_DEFAULT  BLINKT_COLOR_BLUE
#endif

/**
 * @brief  Initializes BLINKT Led
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  clk_port_param: clock signal port
 * @param  clk_pin_param: clock signal pin
 * @param  data_port_param: data signal port
 * @param  data_pin_param: data signal pin
 * @retval None
 */
void BLINKT_Init(uint8_t instance, GPIO_TypeDef *clk_port_param, uint32_t clk_pin_param, GPIO_TypeDef *data_port_param, uint32_t data_pin_param);

/**
 * @brief  Set all Leds ON to the default color
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @retval None
 */
void BLINKT_SetOn(uint8_t instance);

/**
 * @brief  Set all the Leds OFF
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @retval None
 */
void BLINKT_SetOff(uint8_t instance);

/** 
 * @brief  Set the all Leds at specified color
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  color: Color to be set. This parameter must be a value of @ref BLINKT_COLOR_t enumeration
 * @retval None
 */
void BLINKT_SetColor(uint8_t instance, BLINKT_COLOR_t color);

/** 
 * @brief  Set the all Leds at specified color
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  led_mask: Bit mask to select the LEDs to update.
 * @param  color:    Color to be set. This parameter must be a value of @ref BLINKT_COLOR_t enumeration
 * @retval None
 */
void BLINKT_SetLedColor(uint8_t instance, uint16_t led_mask, BLINKT_COLOR_t color);

/**
 * @brief  Set all the Leds at specified red, green and blue colors
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  red:   red level from 0 to 255
 * @param  green: green level from 0 to 255
 * @param  blue:  blue level from 0 to 255
 * @retval None
 */
void BLINKT_SetLevel(uint8_t instance, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief  Set all the Leds at specified red, green and blue colors
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  led_mask: Bit mask to select the LEDs to update.
 * @param  red:      red level from 0 to 255
 * @param  green:    green level from 0 to 255
 * @param  blue:     blue level from 0 to 255
 * @retval None
 */
void BLINKT_SetLedLevel(uint8_t instance, uint16_t led_mask, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief  Set the mode of the bar graph display.
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  mode: mode of the bar graph. This parameter must be a value of @ref BLINKT_BAR_GRAPH_MODE_t enumeration.
 * @retval None
 */
void BLINKT_SetBarGraphMode(uint8_t instance, BLINKT_BAR_GRAPH_MODE_t mode);

/**
 * @brief  Set the level of the bar graph display.
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  level: level value of the bar graph. Must be lower than @ref BAR_GRAPH_LEVEL_MAX.
 * @retval None
 */
void BLINKT_SetBarGraphLevel(uint8_t instance, uint8_t level);

/**
 * @brief  Update the display with rainbow pattern.
 * @note   None
 * @param  instance: instance of he led bar, up to BAR_INSTANCE-1
 * @param  hue_val: HUE val of the rainbow.
 * @param  spacing: spacing in color variation.
 * @retval None
 */
void BLINKT_SetRainbow(uint8_t instance, uint16_t hue_val, uint16_t spacing);

/**
 * @brief  Set the mode of the animation. An animation requires to have 4 instances of LED bar.
 * @note   None
 * @param  mode:     mode of the animation. This parameter must be a value of @ref BLINKT_ANIMATION_MODE_t enumeration.
 * @param  param_a:  paramater A of selected animation
  * @retval None
 */
void BLINKT_SetAnimationMode(BLINKT_ANIMATION_MODE_t mode, uint32_t param_a);

/**
 * @brief  Perform a step on the animation selected.
 * @note   None
 * @param  val_a: gives a value for the curent animation.
 * @retval None
 */
void BLINKT_AnimationStep(uint32_t val_a);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
