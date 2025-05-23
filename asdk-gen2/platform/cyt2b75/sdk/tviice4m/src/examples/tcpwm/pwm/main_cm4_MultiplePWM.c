
/***************************************************************************//**
* \file main_cm4.c
*
* \version 1.0
*
* \brief Main example file for CM4
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"

/* To check this example is functional,
   please see following pin outputs the 976.5625Hz pulse with duty 50%.  */
/* 176 pin MCU on Rev.A CPU board: P16_9, P16_3, P16_4, P16_5 on CPU board. */
   
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/


/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
cy_stc_gpio_pin_config_t pwm_line_out_pin_cfg = 
{
    .outVal    = 0x01u,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom     = HSIOM_SEL_GPIO ,
    .intEdge   = 0u,
    .intMask   = 0u,
    .vtrip     = 0u,
    .slewRate  = 0u,
    .driveSel  = 0u,
    .vregEn    = 0u,
    .ibufMode  = 0u,
    .vtripSel  = 0u,
    .vrefSel   = 0u,
    .vohSel    = 0u,
};

cy_stc_tcpwm_pwm_config_t const MyPWM_config =
{
    .pwmMode            = CY_TCPWM_PWM_MODE_PWM,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_128,		// 2,000,000 / 128 = 15,625Hz
    .debug_pause        = 0uL,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET,
    .underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .deadTime           = 0uL,
    .deadTimeComp       = 0uL,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = 16u - 1,                          // 15,625 / 16 = 976.5625Hz (PWM frequency)
    .period_buff        = 0u,
    .enablePeriodSwap   = false,
    .compare0           = 8u,                               // 8 / 16 = 0.5 (PWM duty)
    .compare1           = 0u,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .invertPWMOut       = 0uL,
    .invertPWMOutN      = 0uL,
    .killMode           = CY_TCPWM_PWM_STOP_ON_KILL,
    .switchInputMode    = CY_TCPWM_INPUT_LEVEL,
    .switchInput        = 0uL,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0uL,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0uL,
    .kill0InputMode     = CY_TCPWM_INPUT_LEVEL,
    .kill0Input         = 0uL,
    .kill1InputMode     = CY_TCPWM_INPUT_LEVEL,
    .kill1Input         = 0uL,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1uL,
};

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
int main(void)
{
    SystemInit();
    
    __enable_irq(); /* Enable global interrupts. */
    
    /*--------------------------------*/
    /* Clock Configuration for TCPWMs */
    /*--------------------------------*/
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT27 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS27, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT28 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS28, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT29 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS29, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT30 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS30, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u);

 
    /*----------------------------*/
    /* Port Configuration for PWM */
    /*----------------------------*/
    pwm_line_out_pin_cfg.hsiom = P3_3_TCPWM0_LINE27;
    Cy_GPIO_Pin_Init(GPIO_PRT3, 3u, &pwm_line_out_pin_cfg); //P3_3_TCPWM_LINE27(For 176pin, P16_9)
    pwm_line_out_pin_cfg.hsiom = P3_4_TCPWM0_LINE28;
    Cy_GPIO_Pin_Init(GPIO_PRT3, 4u, &pwm_line_out_pin_cfg); //P3_4_TCPWM_LINE28(For 176pin, P16_3)
    pwm_line_out_pin_cfg.hsiom = P3_5_TCPWM0_LINE29;
    Cy_GPIO_Pin_Init(GPIO_PRT3, 5u, &pwm_line_out_pin_cfg); //P3_5_TCPWM_LINE29(For 176pin, P16_4)
    pwm_line_out_pin_cfg.hsiom = P4_0_TCPWM0_LINE30;
    Cy_GPIO_Pin_Init(GPIO_PRT4, 0u, &pwm_line_out_pin_cfg); //P4_0_TCPWM_LINE30(For 176pin, P16_6)
    
    /* Initialize TCPWM0_GPR0_CNT27 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT27, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT27);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT27);
    
    /* Initialize TCPWM0_GPR0_CNT28 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT28, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT28);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT28);
    
    /* Initialize TCPWM0_GPR0_CNT29 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT29, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT29);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT29);
    
    /* Initialize TCPWM0_GPR0_CNT30 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT30, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT30);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT30);

    for(;;);
}

/* [] END OF FILE */
