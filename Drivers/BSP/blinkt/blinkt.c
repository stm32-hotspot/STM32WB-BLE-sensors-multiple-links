/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    blinkt.c
  * @author  MCD Application Team
  * @brief   Led bar blinkt driver
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

#include "blinkt.h"
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include <string.h>

#define FRAME_START_SIZE    32
#define LEDX_START_SIZE     3
#define LEDX_BRIGHT_SIZE    5
#define LEDX_BLUE_SIZE      8
#define LEDX_GREEN_SIZE     8
#define LEDX_RED_SIZE       8
#define FRAME_END_SIZE      32
#define LED_LEVEL_MAX       256

#define BIT_1(inst)                                                            \
        do{                                                                    \
          LL_GPIO_SetOutputPin(data_port[inst], data_pin[inst]);               \
          LL_GPIO_SetOutputPin(clk_port[inst], clk_pin[inst]);                 \
          LL_GPIO_ResetOutputPin(clk_port[inst], clk_pin[inst]);               \
        } while(0)

#define BIT_0(inst)                                                            \
        do{                                                                    \
          LL_GPIO_ResetOutputPin(data_port[inst], data_pin[inst]);             \
          LL_GPIO_SetOutputPin(clk_port[inst], clk_pin[inst]);                 \
          LL_GPIO_ResetOutputPin(clk_port[inst], clk_pin[inst]);               \
        } while(0)

struct led_level_t
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};


static void BLINKT_refreshDisplay(uint8_t instance);
static void BLINKT_HSVtoRGB(float h_f, float s_f, float v_f, uint8_t *r_p, uint8_t *g_p, uint8_t *b_p);

/* Private variable */
GPIO_TypeDef *clk_port[BAR_INSTANCE], *data_port[BAR_INSTANCE];
uint32_t clk_pin[BAR_INSTANCE], data_pin[BAR_INSTANCE];
struct led_level_t leds_values[BAR_INSTANCE][LEDS_COUNT];
struct led_level_t leds_values_tmp[BAR_INSTANCE][LEDS_COUNT];
BLINKT_BAR_GRAPH_MODE_t bar_graph_mode[BAR_INSTANCE];
BLINKT_ANIMATION_MODE_t animation_mode;
uint32_t animation_param_a;


void BLINKT_Init(uint8_t instance, GPIO_TypeDef *clk_port_param, uint32_t clk_pin_param, GPIO_TypeDef *data_port_param, uint32_t data_pin_param)
{
  clk_port[instance] = clk_port_param;
  clk_pin[instance] = clk_pin_param;
  
  data_port[instance] = data_port_param;
  data_pin[instance] = data_pin_param;

  bar_graph_mode[instance] = BLINKT_BAR_GRAPH_MODE_BLUE;
  
  animation_mode = BLINKT_ANINATION_MODE_RAINBOW;
  BLINKT_SetOff(instance);
  return;
}

void BLINKT_SetOn(uint8_t instance)
{
  BLINKT_SetLedColor(instance, BLINKT_LED_ALL, BLINKT_COLOR_DEFAULT);
}

void BLINKT_SetOff(uint8_t instance)
{
  BLINKT_SetLedColor(instance, BLINKT_LED_ALL, BLINKT_COLOR_OFF);
}
    

void BLINKT_SetColor(uint8_t instance, BLINKT_COLOR_t color)
{
  BLINKT_SetLedColor(instance, BLINKT_LED_ALL, color);
  return;
}

void BLINKT_SetLevel(uint8_t instance, uint8_t red, uint8_t green, uint8_t blue)
{
  BLINKT_SetLedLevel(instance, BLINKT_LED_ALL, red, green, blue);
  return;
}

void BLINKT_SetLedColor(uint8_t instance, uint16_t led_mask, BLINKT_COLOR_t color) {
  uint8_t n;

  for( n = 0 ; n < LEDS_COUNT ; n++)
  {
    if( (led_mask & (0x0001 << n)) != 0 )
    {
      switch (color)
      {
        case BLINKT_COLOR_OFF:
          leds_values[instance][n].red    = 0;
          leds_values[instance][n].green  = 0;
          leds_values[instance][n].blue   = 0;
          break;
        case BLINKT_COLOR_WHITE:
          leds_values[instance][n].red    = 255;
          leds_values[instance][n].green  = 255;
          leds_values[instance][n].blue   = 255;
          break;
        case BLINKT_COLOR_RED:
          leds_values[instance][n].red    = 255;
          leds_values[instance][n].green  = 0;
          leds_values[instance][n].blue   = 0;
          break;
        case BLINKT_COLOR_GREEN:
          leds_values[instance][n].red    = 0;
          leds_values[instance][n].green  = 255;
          leds_values[instance][n].blue   = 0;
          break;
        case BLINKT_COLOR_BLUE:
          leds_values[instance][n].red    = 0;
          leds_values[instance][n].green  = 0;
          leds_values[instance][n].blue   = 255;
          break;
        default:
          break;
      }
    }
  }
  if( (led_mask & (0x0001 << n)) == 0 )
  {
    BLINKT_refreshDisplay(instance);
  }
  return;
}

void BLINKT_SetLedLevel(uint8_t instance, uint16_t led_mask, uint8_t red, uint8_t green, uint8_t blue) {
  uint8_t n;

  for( n = 0 ; n < LEDS_COUNT ; n++)
  {
    if( (led_mask & (0x0001 << n)) != 0 )
    {
      leds_values[instance][n].red    = red;
      leds_values[instance][n].green  = green;
      leds_values[instance][n].blue   = blue;
    }
  }
  if( (led_mask & (0x0001 << n)) == 0 )
  {
    BLINKT_refreshDisplay(instance);
  }

  return;
}

void BLINKT_SetBarGraphMode(uint8_t instance, BLINKT_BAR_GRAPH_MODE_t mode) {
  if(mode < BLINKT_BAR_GRAPH_MODE_MAX)
  {
    bar_graph_mode[instance] = mode;
  }

  return;
}

void BLINKT_SetBarGraphLevel(uint8_t instance, uint8_t level) {
  uint8_t i, j, n;

  BLINKT_SetLedColor(instance, BLINKT_LED_ALL | BLINKT_NO_REFRESH, BLINKT_COLOR_OFF);
  i = 0;
  j = 0;
  n = LEDS_COUNT - 1;
  while(i < level){
    switch(bar_graph_mode[instance])
    {
    case BLINKT_BAR_GRAPH_MODE_RED:
      leds_values[instance][n].red += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
      break;
    case BLINKT_BAR_GRAPH_MODE_GREEN:
      leds_values[instance][n].green += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
      break;
    case BLINKT_BAR_GRAPH_MODE_BLUE:
      leds_values[instance][n].blue += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
      break;
    case BLINKT_BAR_GRAPH_MODE_COLORS:
      switch(n)
      {
      case 7:
      case 6:
        leds_values[instance][n].blue += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
        break;
      case 5:
      case 4:
        leds_values[instance][n].green += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
        break;
      case 3:
      case 2:
        leds_values[instance][n].green += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT)) / 2;
        leds_values[instance][n].red += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
        break;
      case 1:
      case 0:
        leds_values[instance][n].red += (LED_LEVEL_MAX / (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT));
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }
    i++;
    j++;

    if( j >= (BAR_GRAPH_LEVEL_MAX / LEDS_COUNT) )
    {
      switch(bar_graph_mode[instance])
      {
      case BLINKT_BAR_GRAPH_MODE_RED:
        leds_values[instance][n].red--;
        break;
      case BLINKT_BAR_GRAPH_MODE_GREEN:
        leds_values[instance][n].green--;
        break;
      case BLINKT_BAR_GRAPH_MODE_BLUE:
        leds_values[instance][n].blue--;
        break;
      case BLINKT_BAR_GRAPH_MODE_COLORS:
        switch(n)
        {
        case 7:
        case 6:
          leds_values[instance][n].blue--;
          break;
        case 5:
        case 4:
          leds_values[instance][n].green--;
          break;
        case 3:
        case 2:
          leds_values[instance][n].green--;
          leds_values[instance][n].red--;
          break;
        case 1:
        case 0:
          leds_values[instance][n].red--;
          break;
        default:
          break;
        }
        break;
      default:
        break;
      }

      n--;
      j = 0;
    }
  }

  BLINKT_refreshDisplay(instance);
  return;
}


void BLINKT_SetRainbow(uint8_t instance, uint16_t hue_val, uint16_t spacing) {
  uint8_t n;
  uint32_t hue, offset;
  float h_f;

  //hue = HAL_GetTick() % 360;/* value between 0 and 359 */
  hue = hue_val % 360;/* value between 0 and 359 */

  for( n = 0 ; n < LEDS_COUNT ; n++)
  {
    offset = n * spacing;
    h_f = ((hue + offset) % 360) / 360.0;

    BLINKT_HSVtoRGB(h_f, 1.0, 1.0, &leds_values[instance][n].red, 
                                   &leds_values[instance][n].green, 
                                   &leds_values[instance][n].blue);
  }
  BLINKT_refreshDisplay(instance);

  return;
}

void BLINKT_SetAnimationMode(BLINKT_ANIMATION_MODE_t mode, uint32_t param_a) {
  uint32_t i, n;
  uint8_t r, g, b;
  
  if(mode < BLINKT_ANIMATION_MODE_MAX)
  {
    animation_mode = mode;
    animation_param_a = param_a;
  }

  switch(animation_mode)
  {
    case BLINKT_ANINATION_MODE_ROT_PINK:
    case BLINKT_ANINATION_MODE_ROT_YELLOW:
    case BLINKT_ANINATION_MODE_ROT_BLUE:
    case BLINKT_ANINATION_MODE_ROT_WHITE:
    case BLINKT_ANINATION_MODE_PULSE_WHITE:
      for( i = 0 ; i < BAR_INSTANCE; i++)
      {
        for( n = 0 ; n < LEDS_COUNT ; n++)
        {
          leds_values[i][n].red    = 0;
          leds_values[i][n].green  = 0;
          leds_values[i][n].blue   = 0;
        }
      }
      break; 
    default:
      break;
  }
  
  switch(animation_mode)
  {
    case BLINKT_ANINATION_MODE_ROT_PINK:
      r = 230;
      g = 0;
      b = 120;
      break; 
    case BLINKT_ANINATION_MODE_ROT_YELLOW:
      r = 255;
      g = 210;
      b = 0;
      break; 
    case BLINKT_ANINATION_MODE_ROT_BLUE:
      r = 0;
      g = 0;
      b = 255;
      break; 
    case BLINKT_ANINATION_MODE_ROT_WHITE:
    case BLINKT_ANINATION_MODE_PULSE_WHITE:
      r = 255;
      g = 255;
      b = 255;
      break; 
    default:
      break;
  }
  
  switch(animation_mode)
  {
    case BLINKT_ANINATION_MODE_ROT_PINK:
    case BLINKT_ANINATION_MODE_ROT_YELLOW:
    case BLINKT_ANINATION_MODE_ROT_BLUE:
    case BLINKT_ANINATION_MODE_ROT_WHITE:
      for( n = 0 ; n < LEDS_COUNT ; n++)
      {
        i = n;
        if(param_a == 0)
        {
          i = LEDS_COUNT - 1 - n;
        }          
        leds_values[0][i].red    = r;
        leds_values[0][i].green  = g;
        leds_values[0][i].blue   = b;
      }
      break;
    case BLINKT_ANINATION_MODE_PULSE_WHITE:
      for( n = 0 ; n < LEDS_COUNT ; n++)
      {
        leds_values_tmp[0][n].red = 255 >> n;
      }
      break;
    default:
      break;
  }

  return;
}

void BLINKT_AnimationStep(uint32_t val_a) {
  uint32_t i, n, hue, offset;
  uint32_t i_next, n_next;
  float h_f;
  
  switch(animation_mode)
  {
    case BLINKT_ANINATION_MODE_RAINBOW:
    {      
      hue = val_a % 360;/* value between 0 and 359 */      
      for( i = 0 ; i < BAR_INSTANCE; i++)
      {
        for( n = 0 ; n < LEDS_COUNT ; n++)
        {
          offset = (i + 1) * n * animation_param_a;
          h_f = ((hue + offset) % 360) / 360.0;

          BLINKT_HSVtoRGB(h_f, 1.0, 1.0, &leds_values[i][n].red, 
                                         &leds_values[i][n].green, 
                                         &leds_values[i][n].blue);
        }
      }
      break;
    }
    case BLINKT_ANINATION_MODE_ROT_PINK:
    case BLINKT_ANINATION_MODE_ROT_YELLOW:
    case BLINKT_ANINATION_MODE_ROT_BLUE:
    case BLINKT_ANINATION_MODE_ROT_WHITE:
      memcpy(&leds_values_tmp[0][0], &leds_values[0][0], sizeof(leds_values));
      
      for( i = 0 ; i < BAR_INSTANCE; i++)
      {
        for( n = 0 ; n < LEDS_COUNT ; n++)
        {
          if(val_a == 0)
          {
            i_next = i;
            n_next = n + 1;          
            if(n == (LEDS_COUNT - 1))
            {
              i_next = (i + 1) % BAR_INSTANCE;
              n_next = 0;
            }
          }
          else
          {
            i_next = i;
            n_next = n - 1;          
            if(n == 0)
            {
              if(i == 0)
              {
                i_next = BAR_INSTANCE - 1;
              }
              else
              {
                i_next = i - 1;
              }
              n_next = (LEDS_COUNT - 1);
            }
          }
          leds_values[i_next][n_next].red   = leds_values_tmp[i][n].red;
          leds_values[i_next][n_next].green = leds_values_tmp[i][n].green;
          leds_values[i_next][n_next].blue  = leds_values_tmp[i][n].blue;
        }
      }
      break;
    case BLINKT_ANINATION_MODE_PULSE_WHITE:
      memset(&leds_values[0][0], 0, sizeof(leds_values));
      
      for( i = 0 ; i < BAR_INSTANCE; i++)
      {
        for( n = 0 ; n < (val_a + 1) ; n++)
        {
          leds_values[i][(LEDS_COUNT / 2) - 1 - n].red   = leds_values_tmp[0][val_a + n].red;
          leds_values[i][(LEDS_COUNT / 2) - 1 - n].green = leds_values_tmp[0][val_a + n].green;
          leds_values[i][(LEDS_COUNT / 2) - 1 - n].blue  = leds_values_tmp[0][val_a + n].blue;
            
          leds_values[i][(LEDS_COUNT / 2) + n].red   = leds_values_tmp[0][val_a + n].red;
          leds_values[i][(LEDS_COUNT / 2) + n].green = leds_values_tmp[0][val_a + n].green;
          leds_values[i][(LEDS_COUNT / 2) + n].blue  = leds_values_tmp[0][val_a + n].blue;
        }
      }      
      break;
    default:
      break;
  }
  
  for( i = 0 ; i < BAR_INSTANCE ; i++)
  {
    BLINKT_refreshDisplay(i);
  }

  return;
}

static void BLINKT_refreshDisplay(uint8_t instance) {
  uint8_t i, n;
  
  if( (clk_port[instance] != 0) && (data_port[instance] != 0))
  {
    for( i = 0 ; i < FRAME_START_SIZE ; i++)
    {
      BIT_0(instance);
    }
    for( n = 0 ; n < LEDS_COUNT ; n++)
    {
      for( i = 0 ; i < (LEDX_START_SIZE + LEDX_BRIGHT_SIZE) ; i++)
      {
        BIT_1(instance);
      }
      for( i = 0 ; i < LEDX_BLUE_SIZE ; i++)
      {
        if( (leds_values[instance][n].blue & (0x01 << (LEDX_BLUE_SIZE - 1 - i))) != 0 )
        {
          BIT_1(instance);
        }
        else
        {
          BIT_0(instance);
        }
      }
      for( i = 0 ; i < LEDX_GREEN_SIZE ; i++)
      {
        if( (leds_values[instance][n].green & (0x01 << (LEDX_GREEN_SIZE - 1 - i))) != 0 )
        {
          BIT_1(instance);
        }
        else
        {
          BIT_0(instance);
        }
      }
      for( i = 0 ; i < LEDX_RED_SIZE ; i++)
      {
        if( (leds_values[instance][n].red & (0x01 << (LEDX_RED_SIZE - 1 - i))) != 0 )
        {
          BIT_1(instance);
        }
        else
        {
          BIT_0(instance);
        }
      }
    }
    for( i = 0 ; i < FRAME_END_SIZE ; i++)
    {
      BIT_1(instance);
    }
  }
  return;
}

static void BLINKT_HSVtoRGB(float h_f, float s_f, float v_f, uint8_t *r_p, uint8_t *g_p, uint8_t *b_p) {
  float S, H, V, F, M, N, K;
  float r_f, g_f, b_f;
  uint32_t I;

  S = s_f;  /* Saturation */
  H = h_f;  /* Hue */
  V = v_f;  /* value or brightness */

  if ( S == 0.0 )
  {
    /*
     * Achromatic case, set level of grey
     */
    *r_p = (uint8_t)V;
    *g_p = (uint8_t)V;
    *b_p = (uint8_t)V;
  } else {
    /*
     * Determine levels of primary colours.
     */
    if (H >= 1.0)
    {
       H = 0.0;
    } else {
       H = H * 6;
    }
    I = (int) H;   /* should be in the range 0..5 */
    F = H - I;     /* fractional part */

    M = V * (1 - S);
    N = V * (1 - S * F);
    K = V * (1 - S * (1 - F));

    if (I == 0)
    {
      r_f = V;
      g_f = K;
      b_f = M;
    }
    else if (I == 1)
    {
      r_f = N;
      g_f = V;
      b_f = M;
    }
    else if (I == 2)
    {
      r_f = M;
      g_f = V;
      b_f = K;
    }
    else if (I == 3)
    {
      r_f = M;
      g_f = N;
      b_f = V;
    }
    else if (I == 4)
    {
      r_f = K;
      g_f = M;
      b_f = V;
    }
    else if (I == 5)
    {
      r_f = V;
      g_f = M;
      b_f = N;
    }
    else
    {
      r_f = 0;
      g_f = 0;
      b_f = 0;
    }
  }

  *r_p = (uint8_t)(r_f * 255.0f);
  *g_p = (uint8_t)(g_f * 255.0f);
  *b_p = (uint8_t)(b_f * 255.0f);

  return;
}
