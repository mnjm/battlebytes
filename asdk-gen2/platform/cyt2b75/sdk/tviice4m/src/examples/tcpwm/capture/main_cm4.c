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

  /* If you use 176 pin MCU on Rev.A CPU board, please short P16_3 (Capture) and  P16_9 (PWM) on the base board. */

  /* Capture Mode Configuration def */
  #define TCPWM0_GRPx_CNTx_CAPTURE                TCPWM0_GRP0_CNT26
  #define PCLK_TCPWM0_CLOCKSx_CAPTURE             PCLK_TCPWM0_CLOCKS26
  #define TCPWM_PERI_CLK_DIVIDER_NO_CAPTURE       0u
  
  // TR_ONE_CNT_INx: x = (256 * GrpNum) + (CntNum)
  /* P6_1_TCPWM_TR_ONE_CNT_IN0 */
  #define TCPWM_TR_ONE_CNT_INx_PORT               GPIO_PRT3
  #define TCPWM_TR_ONE_CNT_INx_PIN                4u
  #define TCPWM_TR_ONE_CNT_INx_MUX                P3_4_TCPWM0_TR_ONE_CNT_IN26

  /* PWM Configuration def */
  #define TCPWM0_GRPx_CNTx_PWM                    TCPWM0_GRP0_CNT27
  #define PCLK_TCPWM0_CLOCKSx_PWM                 PCLK_TCPWM0_CLOCKS27
  #define TCPWM_PERI_CLK_DIVIDER_NO_PWM           1u
  
  // LINEx: x = (256 * GrpNum) + (CntNum)
  /* P6_3_TCPWM_LINE1, */
  #define TCPWM_LINEx_PORT                        GPIO_PRT3
  #define TCPWM_LINEx_PIN                         3u
  #define TCPWM_LINEx_MUX                         P3_3_TCPWM0_LINE27

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
cy_stc_gpio_pin_config_t tcpwm_trig_in_pin_cfg = 
{
    .outVal    = 0x0u,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom     = TCPWM_TR_ONE_CNT_INx_MUX,
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

cy_stc_gpio_pin_config_t pwm_line_out_pin_cfg = 
{
    .outVal    = 0x01u,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom     = TCPWM_LINEx_MUX,
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

cy_stc_tcpwm_counter_config_t const MyCounter_config =
{
    .period             = 0xFFFF,                        	// A TCPWM in GRP0 has 16 bit counter
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_4, 		// 2,000,000 / 4 = 500,000 Hz
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = 0uL,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 500uL,
    .compare0_buff      = 32768uL,
    .compare1           = 16384uL,
    .compare1_buff      = 32768uL,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,			// detect rising edge
    .capture0Input      = 2uL,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0uL,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0uL,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0uL,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,          // detect falling edge
    .capture1Input      = 2uL,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1uL,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
	.trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
};

cy_stc_tcpwm_pwm_config_t const MyPWM_config =
{
    .pwmMode            = CY_TCPWM_PWM_MODE_PWM,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .debug_pause        = false,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET,
    .underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .deadTime           = 0uL,
    .deadTimeComp       = 0uL,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = 2560 - 1,                            // Period = 2560 / 2000000 = 1280 [us]
    .period_buff        = 32768uL,
    .enablePeriodSwap   = false,
    .compare0           = 640,                                 // Duty 25% (High duration: 320 [us], Low duration: 960 [us])
    .compare1           = 16384uL,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .invertPWMOut       = 0uL,
    .invertPWMOutN      = 0uL,
    .killMode           = CY_TCPWM_PWM_STOP_ON_KILL,
    .switchInputMode    = CY_TCPWM_INPUT_LEVEL ,
    .switchInput        = 0uL,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL ,
    .reloadInput        = 0uL,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL ,
    .startInput         = 0uL,
    .kill0InputMode     = CY_TCPWM_INPUT_LEVEL ,
    .kill0Input         = 0uL,
    .kill1InputMode     = CY_TCPWM_INPUT_LEVEL ,
    .kill1Input         = 0uL,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL ,
    .countInput         = 1uL,
};

cy_stc_sysint_irq_t irq_cfg = 
{
    .sysIntSrc  = tcpwm_0_interrupts_26_IRQn,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

uint32_t g_periodLow_us  = 0u;
uint32_t g_periodHigh_us = 0u;
uint32_t g_duty          = 0u;

void capture_isr_handler(void)
{
    static uint16_t countCC0Match = 0u;
    static uint16_t countCC1Match = 0u;
    static uint16_t g_countLow    = 0u;
    static uint16_t g_countHigh   = 0u;

    if(Cy_Tcpwm_Counter_GetCC0_IntrMasked(TCPWM0_GRPx_CNTx_CAPTURE))
    {
        // CCO would capture rising edge of input pulse.
        Cy_Tcpwm_Counter_ClearCC0_Intr(TCPWM0_GRPx_CNTx_CAPTURE);

        countCC0Match = Cy_Tcpwm_Counter_GetCompare0(TCPWM0_GRPx_CNTx_CAPTURE);
        if(countCC0Match >= countCC1Match)
        {
            g_countLow = countCC0Match - countCC1Match;
        }
        else
        {
            g_countLow = countCC0Match + (uint16_t)(0x10000 - (uint32_t)countCC1Match);
        }
        g_periodLow_us = g_countLow * 1000000 / 500000;
    }

    if(Cy_Tcpwm_Counter_GetCC1_IntrMasked(TCPWM0_GRPx_CNTx_CAPTURE))
    {
        // CC1 would capture falling edge of input pulse.
        Cy_Tcpwm_Counter_ClearCC1_Intr(TCPWM0_GRPx_CNTx_CAPTURE);

        countCC1Match = Cy_Tcpwm_Counter_GetCompare1(TCPWM0_GRPx_CNTx_CAPTURE);
        if(countCC1Match >= countCC0Match)
        {
            g_countHigh = countCC1Match - countCC0Match;
        }
        else
        {
            g_countHigh = countCC1Match + (uint16_t)(0x10000 - (uint32_t)countCC0Match);
        }
        g_periodHigh_us = g_countHigh * 1000000 / 500000;
    }
    
    g_duty = (g_periodHigh_us * 100) / (g_periodHigh_us + g_periodLow_us);
}

int main(void)
{
    SystemInit();
    
    __enable_irq(); /* Enable global interrupts. */

    /*--------------------------------*/
    /* Clock Configuration for TCPWMs */
    /*--------------------------------*/
    
    /* Assign a programmable divider for PCLK_TCPWM0_CLOCKSx_CAPTURE */    
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKSx_CAPTURE, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_CAPTURE);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_CAPTURE, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_CAPTURE, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif

    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_CAPTURE);

    /* Assign a programmable divider for PCLK_TCPWM0_CLOCKSx_PWM */  
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKSx_PWM, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM);

    /* Sets the 16-bit divider */
  #if (CY_USE_PSVP == 1u)
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM, 11u); // Divider 11 --> 24MHz / (11+1) = 2MHz
  #else
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM, 39u); // Divider 39 --> 80MHz / (39+1) = 2MHz
  #endif    

    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, TCPWM_PERI_CLK_DIVIDER_NO_PWM);


    /*--------------------------------*/
    /* Interrupt setting for Capture  */
    /*--------------------------------*/
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, capture_isr_handler);
    NVIC_EnableIRQ(CPUIntIdx3_IRQn);

    /*------------------------------------------*/
    /* Port Configuration for PWM, Counter Trig */
    /*------------------------------------------*/
    Cy_GPIO_Pin_Init(TCPWM_TR_ONE_CNT_INx_PORT, TCPWM_TR_ONE_CNT_INx_PIN, &tcpwm_trig_in_pin_cfg);
    Cy_GPIO_Pin_Init(TCPWM_LINEx_PORT, TCPWM_LINEx_PIN, &pwm_line_out_pin_cfg);

    /* Initialize PCLK_TCPWM0_CLOCKSx_CAPTURE as Capture Mode & Enable */
    Cy_Tcpwm_Counter_Init(TCPWM0_GRPx_CNTx_CAPTURE, &MyCounter_config);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(TCPWM0_GRPx_CNTx_CAPTURE);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(TCPWM0_GRPx_CNTx_CAPTURE);
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRPx_CNTx_CAPTURE);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRPx_CNTx_CAPTURE); 

    /* Initialize TCPWM0_GRPx_CNTx_PWM as PWM & Enable */
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRPx_CNTx_PWM, &MyPWM_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRPx_CNTx_PWM);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRPx_CNTx_PWM);

    for(;;);
}

/* [] END OF FILE */
