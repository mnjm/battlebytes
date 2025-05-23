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

#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_AB_LED_PORT
    #define USER_LED_PIN            CY_AB_LED_PIN
    #define USER_LED_PIN_MUX        CY_AB_LED_PIN_MUX

    #define USER_BUTTON_PORT        CY_BUTTON1_PORT
    #define USER_BUTTON_PIN         CY_BUTTON1_PIN
    #define USER_BUTTON_PIN_MUX     CY_BUTTON1_PIN_MUX
    #define USER_BUTTON_IRQ         CY_BUTTON1_IRQN
#else
    #define USER_LED_PORT           CY_USER_LED1_PORT
    #define USER_LED_PIN            CY_USER_LED1_PIN
    #define USER_LED_PIN_MUX        CY_USER_LED1_PIN_MUX

    #define USER_BUTTON_PORT        CY_USER_BUTTON_UP_PORT
    #define USER_BUTTON_PIN         CY_USER_BUTTON_UP_PIN
    #define USER_BUTTON_PIN_MUX     CY_USER_BUTTON_UP_PIN_MUX
    #define USER_BUTTON_IRQ         CY_USER_BUTTON_UP_IRQN
#endif

#define CY_HF3_CLK_OUT_PORT     	GPIO_PRT1
#define CY_HF3_CLK_OUT_PIN      	(0ul)
#define CY_HF3_CLK_OUT_PIN_MUX  	P1_0_SRSS_EXT_CLK

cy_stc_gpio_pin_config_t user_led0_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = USER_LED_PIN_MUX ,                           
    .intEdge = 0,                                  
    .intMask = 0,                                  
    .vtrip = 0,                                    
    .slewRate = 0,                                 
    .driveSel = 0,                                 
    .vregEn = 0,                                   
    .ibufMode = 0,                                 
    .vtripSel = 0,                                 
    .vrefSel = 0,                                  
    .vohSel = 0,                                   
};

cy_stc_gpio_pin_config_t user_button3_port_pin_cfg = 
{
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = USER_BUTTON_PIN_MUX ,
    .intEdge = CY_GPIO_INTR_FALLING,
    .intMask = 1,
    .vtrip = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel = 0,
    .vohSel = 0,
};

/******   Clock Output Utilities   ******/
cy_stc_gpio_pin_config_t clkOutPortConfig =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = CY_HF3_CLK_OUT_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* Handler will cause CPU to wake-up from DeepSleep */
void ButtonIntHandler(void)
{
    uint32_t intStatus;

    /* If button falling edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON_PORT, USER_BUTTON_PIN);
    if (intStatus != 0uL)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON_PORT, USER_BUTTON_PIN);
    }
}

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
    
    /* Enable global interrupts. */
    __enable_irq();
    
	/* Enable CM7_0/1. CY_CORTEX_M7_APPL_ADDR is calculated in linker script, check it in case of problems. */
    Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);
    Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR); 

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led0_port_pin_cfg);
    Cy_GPIO_Pin_Init(USER_BUTTON_PORT, USER_BUTTON_PIN, &user_button3_port_pin_cfg);
	
    /* Start output internal clock */
    Cy_GPIO_Pin_Init(CY_HF3_CLK_OUT_PORT, CY_HF3_CLK_OUT_PIN, &clkOutPortConfig);

    /* Setup interrupt for USER_BUTTON */
    cy_stc_sysint_irq_t irq_cfg =
    {
        .sysIntSrc  = USER_BUTTON_IRQ,
        .intIdx     = CPUIntIdx2_IRQn,
        .isEnabled  = true,
    };
    
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, ButtonIntHandler);
    
    NVIC_SetPriority(CPUIntIdx2_IRQn, 3);
    NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
    NVIC_EnableIRQ(CPUIntIdx2_IRQn);
    
    /* Toggle an LED for sometime to overcome the non aacessible state from debugger */
    for(uint16_t i=0; i<100; i++)
    {
        /* Turn on LED for to some time to indicate the DeepSleep mode. */
        Cy_SysTick_DelayInUs(50000);
        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);
    }
	
	Cy_SysPm_RegisterCallback(&myCallback);

    /* Put core into DeepSleep mode, LED will be off. */
    Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

    for(;;)
    {
        /* Turn on LED after waking-up from DeepSleep */
        Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 1);
    }
}



/* [] END OF FILE */
