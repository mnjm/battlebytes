/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
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

#define USER_LED_PORT           CY_BSP_USER_LED1_PORT 
#define USER_LED_PIN            CY_BSP_USER_LED1_PIN
#define USER_LED_PIN_MUX        CY_BSP_USER_LED1_PIN_MUX

/******************************************************************************/
/*                              EXT CLK PORT                                  */
/******************************************************************************/
#define CY_HF1_CLK_OUT_PORT             GPIO_PRT8
#define CY_HF1_CLK_OUT_PIN              (5ul)
#define CY_HF1_CLK_OUT_PIN_MUX          P8_5_SRSS_EXT_CLK

cy_stc_gpio_pin_config_t user_led_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = USER_LED_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/******   Clock Output Utilities   ******/
cy_stc_gpio_pin_config_t clkOutPortConfig =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = CY_HF1_CLK_OUT_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

int main(void)
{
    cy_stc_syspm_callback_params_t myParams; 
    cy_stc_syspm_callback_t myCallback = 
    {
        &Cy_SysClk_DeepSleepCallback,
        CY_SYSPM_DEEPSLEEP,
        CY_SYSPM_CHECK_FAIL,
        &myParams,
        NULL,
        NULL
    };
	
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */

    /* Enable CM4. CY_CORTEX_M4_APPL_ADDR is calculated in linker script, check it in case of problems. */
    Cy_SysEnableApplCore(CY_CORTEX_M4_APPL_ADDR);

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led_port_pin_cfg);

    /* Start output internal clock */
    Cy_GPIO_Pin_Init(CY_HF1_CLK_OUT_PORT, CY_HF1_CLK_OUT_PIN, &clkOutPortConfig);
	
	Cy_SysPm_RegisterCallback(&myCallback);

    /* Put the system to DeepSleep */
    Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

    for(;;)
    {
        Cy_SysTick_DelayInUs(50000);
        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);
    }
}



/* [] END OF FILE */
