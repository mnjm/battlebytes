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

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/* To check this example is functional,
   please see following pin outputs the 200Hz with right side dead time 250 [us] and left side dead time 500 [us] */
/* 176 pin MCU: R1011_2 (PWM) and P17_8 (PWM COMPL) on the CPU board. */

/* PWM DT Mode Configuration def */
  
#define TCPWMx_GRPx_CNTx_PWM_DT                 TCPWM0_GRP1_CNT0
#define PCLK_TCPWMx_CLOCKSx_PWM_DT              PCLK_TCPWM0_CLOCKS256
#define TCPWM_PERI_CLK_DIVIDER_NO_PWM_DT        0u

// LINEx: x = (256 * GrpNum) + (CntNum)
/* TCPWM_LINE */
#define TCPWMx_LINEx_PORT                       GPIO_PRT5
#define TCPWMx_LINEx_PIN                        0u
#define TCPWMx_LINEx_MUX                        P5_0_TCPWM0_LINE256

// LINE_COMPLx: x = (256 * GrpNum) + (CntNum)
/* TCPWM_LINE_COMPL */
#define TCPWMx_LINE_CMPLx_PORT                  GPIO_PRT5
#define TCPWMx_LINE_CMPLx_PIN                   1u
#define TCPWMx_LINE_CMPLx_MUX                   P5_1_TCPWM0_LINE_COMPL256

/* PWM_DT Period & DT configurations */
#define TCPWMx_PERIOD                           10000ul // 2,000,000 / 10000 = 200Hz
#define TCPWMx_COMPARE0                         5000ul  // 5000 / 10000 = 0.5 (duty)
#define TCPWMx_DEADTIME                         500ul   // Right side dead time: 500 * (1 / 2,000,000) = 250 us
#define TCPWMx_DEADTIME_COMPL                   1000ul  // Left side dead time :1000 * (1 / 2,000,000) = 500 us

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
    .hsiom     = TCPWMx_LINEx_MUX,
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
    .pwmMode            = CY_TCPWM_PWM_MODE_DEADTIME,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .debug_pause        = false,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET,
    .underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .deadTime           = TCPWMx_DEADTIME,
    .deadTimeComp       = TCPWMx_DEADTIME_COMPL,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = TCPWMx_PERIOD - 1ul,
    .period_buff        = 32768uL,
    .enablePeriodSwap   = false,
    .compare0           = TCPWMx_COMPARE0,
    .compare1           = 16384uL,
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
    Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_2); //CLK_HF2 enable
    
    /*--------------------------------*/
    /* Clock Configuration for TCPWMs */
    /*--------------------------------*/
    
    /* Assign a programmable divider  for TCPWM0_GRPx_CNTx_PWM_PR */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWMx_CLOCKSx_PWM_DT, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM_DT);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM_DT, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM_DT, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM_DT);

    /*------------------------------------------------*/
    /* Port Configuration for TCPWM DT, LEDs (GPIOs)  */
    /*------------------------------------------------*/    
    pwm_line_out_pin_cfg.hsiom = TCPWMx_LINEx_MUX;
    Cy_GPIO_Pin_Init(TCPWMx_LINEx_PORT, TCPWMx_LINEx_PIN, &pwm_line_out_pin_cfg);
    pwm_line_out_pin_cfg.hsiom = TCPWMx_LINE_CMPLx_MUX;
    Cy_GPIO_Pin_Init(TCPWMx_LINE_CMPLx_PORT, TCPWMx_LINE_CMPLx_PIN, &pwm_line_out_pin_cfg);
    
    /* Initialize TCPWMx_GRPx_CNTx_PWM_PR as PWM-DT Mode & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWMx_GRPx_CNTx_PWM_DT, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWMx_GRPx_CNTx_PWM_DT);
    Cy_Tcpwm_TriggerStart(TCPWMx_GRPx_CNTx_PWM_DT);

    for(;;);
}

/* [] END OF FILE */
