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
   please see following pin outputs the 20000Hz pulse with duty 50%.  */
/* 176 pin MCU on Rev.A CPU board: J2.10,  J10.7,  J2.16, J6.11, J5.18,  J6.7, J2.14, J6.14, J6.15, J6.12 on the base board. */
/* 176 pin MCU on Rev.C CPU board: J2.10,  J2.13,  J9.4, J1.4, J11.3,  J10.15, J12.6, J6.14, J6.15, J6.12 on the base board. */
/*  64 pin MCU: J2.10, J10.10, J10.12,  J1.4,  J1.9, J1.10, J2.14, J6.14, J6.15, J6.12 on the base board. */

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

cy_stc_tcpwm_pwm_config_t MyPWM_config =
{
    .pwmMode            = CY_TCPWM_PWM_MODE_PWM,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1, 		// 2,000,000 / 1 = 2,000,000Hz
    .debug_pause        = 0uL,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET,
    .underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .deadTime           = 0uL,
    .deadTimeComp       = 0uL,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = 100 - 1,                        	// 2,000,000 / 100 = 20,000Hz
    .period_buff        = 0,
    .enablePeriodSwap   = false,
    .compare0           = 50,
    .compare1           = 0,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .invertPWMOut       = 0uL,
    .invertPWMOutN      = 0uL,
    .killMode           = CY_TCPWM_PWM_SYNCH_STOP_ON_KILL,	// Kill event stops the counter, synchronous kill mode
    .switchInputMode    = CY_TCPWM_INPUT_LEVEL,
    .switchInput        = 0uL,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0uL,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0uL,
    .kill0InputMode     = CY_TCPWM_INPUT_LEVEL,
    .kill0Input         = 5uL,                            // Initialize counter for kill on tr_all_cnt_in[0] 
    .kill1InputMode     = CY_TCPWM_INPUT_LEVEL,
    .kill1Input         = 0uL,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1uL,
};

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/


void generic_isr_handler(void)
{
    /* ISR Handler */
}

int main(void)
{
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */
    
    /*--------------------------------*/
    /* Clock Configuration for TCPWMs */
    /*--------------------------------*/

    /* Assign a programmable divider for TCPWM0_GRP0_CNT0 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS0, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT1 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS1, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 1u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT2 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS2, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 2u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT6 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS6, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 3u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT9 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS9, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 4u);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)    
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 4u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 4u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 4u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT10 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS10, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 5u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 5u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 5u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 5u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT15 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS15, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 6u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 6u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 6u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 6u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT17 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS17, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 7u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 7u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 7u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 7u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT18 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS18, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 8u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 8u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 8u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 8u);
    
    
    /* Assign a programmable divider for TCPWM0_GRP0_CNT19 */
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS19, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 9u);
    
    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 9u, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 9u, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 9u);

    /*---------------------------------------------*/
    /* Port Configuration for PWM, LEDs (GPIOs)    */
    /*---------------------------------------------*/
    pwm_line_out_pin_cfg.hsiom = P6_1_TCPWM0_LINE0;
    Cy_GPIO_Pin_Init(GPIO_PRT6, 1u, &pwm_line_out_pin_cfg); //P6_1_TCPWM_LINE0
    pwm_line_out_pin_cfg.hsiom = P6_3_TCPWM0_LINE1;
    Cy_GPIO_Pin_Init(GPIO_PRT6, 3u, &pwm_line_out_pin_cfg); //P6_3_TCPWM_LINE1
    pwm_line_out_pin_cfg.hsiom = P6_5_TCPWM0_LINE2;
    Cy_GPIO_Pin_Init(GPIO_PRT6, 5u, &pwm_line_out_pin_cfg); //P6_5_TCPWM_LINE2
    pwm_line_out_pin_cfg.hsiom = P2_1_TCPWM0_LINE6;
    Cy_GPIO_Pin_Init(GPIO_PRT2, 1u, &pwm_line_out_pin_cfg); //P2_1_TCPWM_LINE6
    pwm_line_out_pin_cfg.hsiom = P5_0_TCPWM0_LINE9;
    Cy_GPIO_Pin_Init(GPIO_PRT5, 0u, &pwm_line_out_pin_cfg); //P5_0_TCPWM_LINE9
    pwm_line_out_pin_cfg.hsiom = P5_1_TCPWM0_LINE10;
    Cy_GPIO_Pin_Init(GPIO_PRT5, 1u, &pwm_line_out_pin_cfg); //P5_1_TCPWM_LINE10
    pwm_line_out_pin_cfg.hsiom = P7_1_TCPWM0_LINE15;
    Cy_GPIO_Pin_Init(GPIO_PRT7, 1u, &pwm_line_out_pin_cfg); //P7_1_TCPWM_LINE15
    pwm_line_out_pin_cfg.hsiom = P0_1_TCPWM0_LINE17;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 1u, &pwm_line_out_pin_cfg); //P0_1_TCPWM_LINE17
    pwm_line_out_pin_cfg.hsiom = P0_0_TCPWM0_LINE18;
    Cy_GPIO_Pin_Init(GPIO_PRT0, 0u, &pwm_line_out_pin_cfg); //P0_0_TCPWM_LINE18
    pwm_line_out_pin_cfg.hsiom = P8_0_TCPWM0_LINE19;
    Cy_GPIO_Pin_Init(GPIO_PRT8, 0u, &pwm_line_out_pin_cfg); //P8_0_TCPWM_LINE19

    /* Initialize TCPWM0_GPR0_CNT0 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT0, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT0);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT0);
    
    /* Initialize TCPWM0_GPR0_CNT1 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT1, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT1);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT1);
    
    /* Initialize TCPWM0_GPR0_CNT2 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT2, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT2);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT2);
    
    /* Initialize TCPWM0_GPR0_CNT6 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT6, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT6);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT6);
    
    /* Initialize TCPWM0_GPR0_CNT9 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT9, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT9);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT9);
    
    /* Initialize TCPWM0_GPR0_CNT10 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT10, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT10);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT10);
    
    /* Initialize TCPWM0_GPR0_CNT15 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT15, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT15);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT15);
    
    /* Initialize TCPWM0_GPR0_CNT17 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT17, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT17);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT17);
    
    /* Initialize TCPWM0_GPR0_CNT18 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT18, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT18);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT18);
    
    /* Initialize TCPWM0_GPR0_CNT19 as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT19, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT19);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT19);

    // Wait 0.05 [s]
    Cy_SysTick_DelayInUs(50000);
    
    /*Trigger tr_all_cnt_in[0] by SW*/
    Cy_TrigMux_SwTrigger(TRIG_OUT_MUX_4_TCPWM_ALL_CNT_TR_IN0, TRIGGER_TYPE_LEVEL, 1 /*output*/); 

    for(;;);
}

/* [] END OF FILE */
